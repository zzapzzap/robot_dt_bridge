using UnityEngine;

namespace SierraBase.RobotDT
{
    /// <summary>
    /// 씬 전체가 공유하는 접속 · 토픽 설정.
    /// 빈 GameObject 하나에 붙여 두고, 다른 스크립트는 여기서 값을 읽는다.
    /// ROS 2 쪽 config/robots.yaml 의 topics 항목과 이름이 일치해야 한다.
    /// </summary>
    [DisallowMultipleComponent]
    public class DtBridgeConfig : MonoBehaviour
    {
        public static DtBridgeConfig Instance { get; private set; }

        [Header("ROS-TCP-Endpoint")]
        [Tooltip("Host AGX Orin 의 IP. 개발 중에는 127.0.0.1")]
        public string rosIpAddress = "127.0.0.1";
        public int rosPort = 10000;

        [Header("로봇 토픽 (robots.yaml 과 동일해야 함)")]
        public string robotId = "loading";
        public string poseTopic = "/robot/loading/cmd_degs";      // Float64MultiArray[6]
        public string stateTopic = "/robot/loading/state";        // Int32MultiArray[7]
        public string commandTopic = "/robot/loading/unity_command"; // Int32MultiArray[6]

        [Header("작업자 토픽")]
        public string workerTopic = "/worker/unity/bodies";       // Float32MultiArray
        public int workerJoints = 28;
        public int maxBodies = 5;

        [Header("표시 옵션")]
        [Tooltip("수신이 끊겼을 때 마지막 자세를 유지할 시간(초). 넘으면 회색 처리")]
        public float staleTimeout = 0.5f;
        [Tooltip("각도 보간 계수. 0 이면 보간 없음(원시값 그대로)")]
        [Range(0f, 1f)] public float smoothing = 0.35f;

        void Awake()
        {
            if (Instance != null && Instance != this) { Destroy(gameObject); return; }
            Instance = this;
        }

        /// <summary>state 배열 인덱스 — unity_adapter 의 state_layout 순서.</summary>
        public static class StateIdx
        {
            public const int Run = 0;
            public const int Hold = 1;
            public const int EmergencyStop = 2;
            public const int SpeedDown1 = 3;   // 25 % 감속 → 속도 75 %
            public const int SpeedDown2 = 4;   // 50 % 감속 → 속도 50 %
            public const int SpeedDown3 = 5;   // 75 % 감속 → 속도 25 %
            public const int OperationState = 6;
            public const int Length = 7;
        }

        /// <summary>command 배열 인덱스 — unity_adapter 의 command_layout 순서.</summary>
        public static class CmdIdx
        {
            public const int Run = 0;
            public const int Hold = 1;
            public const int Stop = 2;
            public const int SpeedDown1 = 3;   // 속도 75 %
            public const int SpeedDown2 = 4;   // 속도 50 %
            public const int SpeedDown3 = 5;   // 속도 25 %
            public const int Length = 6;
        }
    }
}
