using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

namespace SierraBase.RobotDT
{
    /// <summary>
    /// 씬을 코드로 최소 구성한다 — 공장 배경 없이 <b>바닥 그리드 + 로봇 + 작업자</b> 만.
    ///
    /// 빈 씬에 이 스크립트 하나만 붙여 재생하면 다음이 자동 생성된다.
    ///   · ROSConnection (IP/포트는 DtBridgeConfig 값)
    ///   · 6축 로봇 (도면 미배치 시 프리미티브 대체 모델)
    ///   · 작업자 스켈레톤 수신기
    ///   · 조작 패널 GUI
    ///
    /// 실제 STEP 모델을 쓸 때는 robotRoot 에 임포트한 모델을 꽂고
    /// autoBuildPlaceholder 를 끄면 된다.
    /// </summary>
    [DisallowMultipleComponent]
    public class DtSceneBootstrap : MonoBehaviour
    {
        [Header("설정")]
        public DtBridgeConfig config;

        [Header("로봇")]
        [Tooltip("실제 모델을 쓸 때 여기에 루트를 꽂는다")]
        public Transform robotRoot;
        [Tooltip("모델이 없으면 원통 6축 대체 모델을 만든다")]
        public bool autoBuildPlaceholder = true;

        [Header("바닥")]
        public bool drawGrid = true;
        public float gridSize = 8f;
        public float gridStep = 0.5f;

        RobotPoseReceiver _pose;
        RobotStateReceiver _state;

        void Awake()
        {
            if (config == null) config = GetComponent<DtBridgeConfig>() ?? DtBridgeConfig.Instance;
            if (config == null) config = gameObject.AddComponent<DtBridgeConfig>();
        }

        void Start()
        {
            var ros = ROSConnection.GetOrCreateInstance();
            ros.RosIPAddress = config.rosIpAddress;
            ros.RosPort = config.rosPort;
            ros.Connect();

            if (robotRoot == null && autoBuildPlaceholder) robotRoot = BuildPlaceholderRobot();
            if (drawGrid) BuildGrid();

            var worker = new GameObject("Workers").AddComponent<WorkerPoseReceiver>();
            worker.config = config;
            worker.robotOrigin = robotRoot;

            var sender = gameObject.AddComponent<SafetyCommandSender>();
            sender.config = config;

            if (Camera.main == null)
            {
                var camGo = new GameObject("Main Camera") { tag = "MainCamera" };
                var cam = camGo.AddComponent<Camera>();
                cam.transform.position = new Vector3(4.5f, 3.2f, -4.5f);
                cam.transform.LookAt(new Vector3(0f, 1.0f, 0f));
                camGo.AddComponent<OrbitCamera>().target = robotRoot;
            }
            if (FindObjectOfType<Light>() == null)
            {
                var l = new GameObject("Sun").AddComponent<Light>();
                l.type = LightType.Directional;
                l.intensity = 1.1f;
                l.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
            }

            Debug.Log($"[Bootstrap] ROS {config.rosIpAddress}:{config.rosPort} · 로봇 {config.robotId}");
        }

        // ------------------------------------------------------------ 대체 모델
        Transform BuildPlaceholderRobot()
        {
            var root = new GameObject("Robot_Placeholder").transform;

            // base
            var basePlate = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            basePlate.name = "Base";
            basePlate.transform.SetParent(root, false);
            basePlate.transform.localScale = new Vector3(0.45f, 0.06f, 0.45f);

            // 링크 6개 — 실제 YS080 비율에 대충 맞춘 길이
            float[] len = { 0.35f, 0.55f, 0.45f, 0.30f, 0.20f, 0.12f };
            float[] rad = { 0.16f, 0.13f, 0.11f, 0.09f, 0.07f, 0.06f };
            Vector3[] axes = {
                Vector3.up, Vector3.right, Vector3.right,
                Vector3.up, Vector3.right, Vector3.up
            };

            var joints = new RobotPoseReceiver.Joint[6];
            var renderers = new System.Collections.Generic.List<Renderer>();
            Transform parent = root;
            float y = 0.06f;

            for (int i = 0; i < 6; i++)
            {
                var pivot = new GameObject($"J{i + 1}").transform;
                pivot.SetParent(parent, false);
                pivot.localPosition = new Vector3(0f, y, 0f);

                var link = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
                link.name = $"Link{i + 1}";
                link.transform.SetParent(pivot, false);
                link.transform.localPosition = new Vector3(0f, len[i] * 0.5f, 0f);
                link.transform.localScale = new Vector3(rad[i] * 2f, len[i] * 0.5f, rad[i] * 2f);
                Destroy(link.GetComponent<Collider>());
                renderers.Add(link.GetComponent<Renderer>());

                joints[i] = new RobotPoseReceiver.Joint
                {
                    name = $"J{i + 1}",
                    link = pivot,
                    axis = axes[i],
                    sign = 1f,
                    offsetDeg = 0f,
                };
                parent = pivot;
                y = len[i];
            }

            _pose = root.gameObject.AddComponent<RobotPoseReceiver>();
            _pose.config = config;
            _pose.joints = joints;

            _state = root.gameObject.AddComponent<RobotStateReceiver>();
            _state.config = config;
            _state.robotRenderers = renderers.ToArray();

            var beaconGo = new GameObject("Beacon");
            beaconGo.transform.SetParent(root, false);
            beaconGo.transform.localPosition = new Vector3(0.6f, 1.8f, 0f);
            var beacon = beaconGo.AddComponent<Light>();
            beacon.type = LightType.Point;
            beacon.range = 6f;
            _state.beacon = beacon;

            return root;
        }

        void BuildGrid()
        {
            var go = new GameObject("Grid");
            go.transform.SetParent(transform, false);
            var lr = go.AddComponent<LineRenderer>();
            lr.useWorldSpace = true;
            lr.widthMultiplier = 0.006f;
            lr.material = new Material(Shader.Find("Sprites/Default"));
            lr.startColor = lr.endColor = new Color(0.55f, 0.6f, 0.7f, 0.35f);

            var pts = new System.Collections.Generic.List<Vector3>();
            for (float x = -gridSize; x <= gridSize + 1e-3f; x += gridStep)
            {
                pts.Add(new Vector3(x, 0, -gridSize));
                pts.Add(new Vector3(x, 0, gridSize));
                pts.Add(new Vector3(x, 0, -gridSize));
            }
            for (float z = -gridSize; z <= gridSize + 1e-3f; z += gridStep)
            {
                pts.Add(new Vector3(-gridSize, 0, z));
                pts.Add(new Vector3(gridSize, 0, z));
                pts.Add(new Vector3(-gridSize, 0, z));
            }
            lr.positionCount = pts.Count;
            lr.SetPositions(pts.ToArray());
        }
    }

    /// <summary>마우스 드래그로 도는 최소 카메라.</summary>
    public class OrbitCamera : MonoBehaviour
    {
        public Transform target;
        public float distance = 6f, yaw = 35f, pitch = 22f;
        public float sensitivity = 3f, zoomSpeed = 2f;

        void LateUpdate()
        {
            if (Input.GetMouseButton(1))
            {
                yaw += Input.GetAxis("Mouse X") * sensitivity;
                pitch = Mathf.Clamp(pitch - Input.GetAxis("Mouse Y") * sensitivity, -5f, 80f);
            }
            distance = Mathf.Clamp(distance - Input.mouseScrollDelta.y * zoomSpeed, 1.5f, 25f);
            Vector3 pivot = target != null ? target.position + Vector3.up * 0.8f : Vector3.up * 0.8f;
            var rot = Quaternion.Euler(pitch, yaw, 0f);
            transform.position = pivot + rot * new Vector3(0f, 0f, -distance);
            transform.LookAt(pivot);
        }
    }
}
