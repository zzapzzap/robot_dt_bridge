using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosFloat32MultiArray = RosMessageTypes.Std.Float32MultiArrayMsg;

namespace SierraBase.RobotDT
{
    /// <summary>
    /// 작업자 28관절 수신 — /worker/unity/bodies (Float32MultiArray).
    ///
    /// 배열 형식 (unity_adapter 와 동일)
    ///     [n, id0, x,y,z ×28, id1, x,y,z ×28, …]
    ///
    /// 공장 배경 없이 <b>관절 구(sphere) + 뼈대 선</b> 만 그린다.
    /// 프리팹 없이도 런타임에 생성되므로 씬이 가볍다.
    /// </summary>
    public class WorkerPoseReceiver : MonoBehaviour
    {
        [Header("설정")]
        public DtBridgeConfig config;

        [Header("표현")]
        public float jointRadius = 0.055f;
        public float boneWidth = 0.022f;
        public Color bodyColor = new Color(0.20f, 0.45f, 0.95f);
        public Color dangerColor = new Color(0.95f, 0.25f, 0.20f);
        [Tooltip("로봇 원점. 이격거리 계산에 쓰며 비워도 동작한다")]
        public Transform robotOrigin;
        [Tooltip("이 거리보다 가까우면 작업자를 빨갛게 표시(m)")]
        public float dangerDistance = 1.5f;

        [Header("좌표 변환")]
        [Tooltip("ROS(오른손 Z-up) → Unity(왼손 Y-up) 변환 적용")]
        public bool convertRosToUnity = true;
        public Vector3 worldOffset = Vector3.zero;

        // ROS 28관절 인덱스 기준 뼈대 연결 (multiview_pose 정의)
        static readonly int[,] BONES = {
            {0,1},{1,2},{1,3},{2,4},{3,5},{4,6},{5,7},
            {1,18},{18,8},{18,9},{8,10},{9,11},{10,12},{11,13},
            {12,14},{13,15},{12,16},{13,17},
            {1,23},{23,0},{0,24},{0,25},{24,26},{25,27},
            {8,20},{9,21},{18,22},{19,1}
        };

        class Body
        {
            public GameObject root;
            public Transform[] joints;
            public Transform[] bones;
            public float lastSeen;
        }

        readonly Dictionary<int, Body> _bodies = new();
        float _lastRecv = -999f;
        Material _mat;

        void Start()
        {
            if (config == null) config = DtBridgeConfig.Instance;
            if (config == null) { enabled = false; return; }

            var shader = Shader.Find("Universal Render Pipeline/Lit")
                         ?? Shader.Find("HDRP/Lit")
                         ?? Shader.Find("Standard");
            _mat = new Material(shader);

            ROSConnection.GetOrCreateInstance()
                .Subscribe<RosFloat32MultiArray>(config.workerTopic, OnBodies);
            Debug.Log($"[WorkerPoseReceiver] 구독 : {config.workerTopic}");
        }

        void OnBodies(RosFloat32MultiArray msg)
        {
            var d = msg.data;
            if (d == null || d.Length < 1) return;
            _lastRecv = Time.time;

            int n = Mathf.RoundToInt(d[0]);
            int stride = 1 + config.workerJoints * 3;     // id + 관절
            int p = 1;
            var seen = new HashSet<int>();

            for (int b = 0; b < n && p + stride <= d.Length; b++)
            {
                int id = Mathf.RoundToInt(d[p]);
                seen.Add(id);
                var body = GetOrCreate(id);
                for (int j = 0; j < config.workerJoints; j++)
                {
                    int o = p + 1 + j * 3;
                    body.joints[j].localPosition = ToUnity(d[o], d[o + 1], d[o + 2]);
                }
                body.lastSeen = Time.time;
                p += stride;
            }

            foreach (var kv in _bodies)
                if (!seen.Contains(kv.Key)) kv.Value.root.SetActive(false);
        }

        Vector3 ToUnity(float x, float y, float z)
        {
            // ROS  : x 앞, y 왼쪽, z 위
            // Unity: x 오른쪽, y 위, z 앞
            var v = convertRosToUnity ? new Vector3(-y, z, x) : new Vector3(x, y, z);
            return v + worldOffset;
        }

        Body GetOrCreate(int id)
        {
            if (_bodies.TryGetValue(id, out var b)) { b.root.SetActive(true); return b; }

            var root = new GameObject($"Worker_{id}");
            root.transform.SetParent(transform, false);

            var joints = new Transform[config.workerJoints];
            for (int j = 0; j < joints.Length; j++)
            {
                var s = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                s.name = $"J{j:00}";
                Destroy(s.GetComponent<Collider>());
                s.transform.SetParent(root.transform, false);
                s.transform.localScale = Vector3.one * (jointRadius * 2f);
                s.GetComponent<Renderer>().sharedMaterial = _mat;
                joints[j] = s.transform;
            }

            int nb = BONES.GetLength(0);
            var bones = new Transform[nb];
            for (int k = 0; k < nb; k++)
            {
                var c = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                c.name = $"B{k:00}";
                Destroy(c.GetComponent<Collider>());
                c.transform.SetParent(root.transform, false);
                c.GetComponent<Renderer>().sharedMaterial = _mat;
                bones[k] = c.transform;
            }

            b = new Body { root = root, joints = joints, bones = bones, lastSeen = Time.time };
            _bodies[id] = b;
            return b;
        }

        void LateUpdate()
        {
            bool alive = (Time.time - _lastRecv) <= config.staleTimeout;

            foreach (var kv in _bodies)
            {
                var body = kv.Value;
                if (!body.root.activeSelf) continue;
                if (!alive) { body.root.SetActive(false); continue; }

                // 뼈대 갱신
                int nb = BONES.GetLength(0);
                for (int k = 0; k < nb; k++)
                {
                    int a = BONES[k, 0], c = BONES[k, 1];
                    if (a >= body.joints.Length || c >= body.joints.Length) continue;
                    Vector3 pa = body.joints[a].localPosition;
                    Vector3 pc = body.joints[c].localPosition;
                    Vector3 mid = (pa + pc) * 0.5f;
                    Vector3 dir = pc - pa;
                    float len = dir.magnitude;
                    var t = body.bones[k];
                    if (len < 1e-4f) { t.gameObject.SetActive(false); continue; }
                    t.gameObject.SetActive(true);
                    t.localPosition = mid;
                    t.localRotation = Quaternion.FromToRotation(Vector3.up, dir.normalized);
                    t.localScale = new Vector3(boneWidth, len * 0.5f, boneWidth);
                }

                // 이격거리에 따른 색
                Color c2 = bodyColor;
                if (robotOrigin != null)
                {
                    float dmin = float.MaxValue;
                    foreach (var j in body.joints)
                        dmin = Mathf.Min(dmin, Vector3.Distance(j.position, robotOrigin.position));
                    if (dmin < dangerDistance) c2 = dangerColor;
                }
                foreach (var j in body.joints)
                    j.GetComponent<Renderer>().material.color = c2;
            }
        }
    }
}
