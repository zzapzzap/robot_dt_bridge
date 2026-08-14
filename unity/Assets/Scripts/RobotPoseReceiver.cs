using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosFloat64MultiArray = RosMessageTypes.Std.Float64MultiArrayMsg;

namespace SierraBase.RobotDT
{
    /// <summary>
    /// 메신저 ① 수신부 — /robot/&lt;id&gt;/cmd_degs (Float64MultiArray[6], degree) 를
    /// 구독해 6축 관절에 그대로 적용한다.
    ///
    /// 관절 6개를 base → J1 → … → J6 순서로 Inspector 에 꽂아 두면 된다.
    /// 회전축과 부호는 STEP 도면(YS080/HH050)에서 읽은 값을 기본으로 하며,
    /// 실로봇과 방향이 반대면 sign 을 -1 로 뒤집는다.
    /// </summary>
    public class RobotPoseReceiver : MonoBehaviour
    {
        [Serializable]
        public class Joint
        {
            public string name = "J1";
            [Tooltip("해당 축을 회전시킬 Transform")]
            public Transform link;
            [Tooltip("로컬 회전축 (STEP 기준)")]
            public Vector3 axis = Vector3.up;
            [Tooltip("PLC 부호 ↔ Unity 회전방향 정합. 어긋나면 -1")]
            public float sign = 1f;
            [Tooltip("모델 home 자세 보정(도)")]
            public float offsetDeg = 0f;

            [HideInInspector] public Quaternion home;
        }

        [Header("설정")]
        public DtBridgeConfig config;

        [Header("관절 6축 — base 에서 손끝 순서")]
        public Joint[] joints = new Joint[6];

        [Header("상태 (읽기 전용)")]
        [SerializeField] private double[] lastDegrees = new double[6];
        [SerializeField] private float lastRecvTime = -999f;
        [SerializeField] private bool linkAlive;

        private double[] _target = new double[6];
        private double[] _shown = new double[6];
        private bool _hasData;

        public bool LinkAlive => linkAlive;
        public double[] Degrees => lastDegrees;

        void Start()
        {
            if (config == null) config = DtBridgeConfig.Instance;
            if (config == null)
            {
                Debug.LogError("[RobotPoseReceiver] DtBridgeConfig 를 찾을 수 없습니다.");
                enabled = false;
                return;
            }

            foreach (var j in joints)
                if (j != null && j.link != null) j.home = j.link.localRotation;

            ROSConnection.GetOrCreateInstance()
                .Subscribe<RosFloat64MultiArray>(config.poseTopic, OnPose);

            Debug.Log($"[RobotPoseReceiver] 구독 : {config.poseTopic}");
        }

        void OnPose(RosFloat64MultiArray msg)
        {
            if (msg.data == null || msg.data.Length < joints.Length) return;
            for (int i = 0; i < joints.Length; i++) _target[i] = msg.data[i];
            lastDegrees = (double[])_target.Clone();
            lastRecvTime = Time.time;
            if (!_hasData) { Array.Copy(_target, _shown, _target.Length); _hasData = true; }
        }

        void Update()
        {
            linkAlive = _hasData && (Time.time - lastRecvTime) <= config.staleTimeout;
            if (!_hasData) return;

            float k = config.smoothing <= 0f
                ? 1f
                : 1f - Mathf.Exp(-Time.deltaTime / Mathf.Max(config.smoothing * 0.1f, 1e-4f));

            for (int i = 0; i < joints.Length; i++)
            {
                var j = joints[i];
                if (j == null || j.link == null) continue;

                _shown[i] += (_target[i] - _shown[i]) * k;
                float deg = (float)_shown[i] * j.sign + j.offsetDeg;
                j.link.localRotation = j.home * Quaternion.AngleAxis(deg, j.axis.normalized);
            }
        }
    }
}
