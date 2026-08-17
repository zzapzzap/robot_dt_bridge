using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosInt32MultiArray = RosMessageTypes.Std.Int32MultiArrayMsg;

namespace SierraBase.RobotDT
{
    /// <summary>
    /// 메신저 ① 송신부 — Unity 조작 패널 → /robot/&lt;id&gt;/unity_command.
    ///
    /// 발행된 명령은 ROS 2 쪽 unity_adapter → safety_gate 를 거쳐 중재된 뒤
    /// PLC 버퍼(D2000 / D3000)에 기록된다. Unity 명령의 우선순위는 0 이므로
    /// XDI(정지) · XAG(속도제한) 지령을 덮어쓰지 못한다 — 안전상 의도된 설계다.
    ///
    /// 검증용 GUI 는 OnGUI 로 그린다(캔버스 셋업 불필요).
    /// </summary>
    public class SafetyCommandSender : MonoBehaviour
    {
        [Header("설정")]
        public DtBridgeConfig config;

        [Header("자동 재발행")]
        [Tooltip("safety_gate 의 command_timeout(기본 300 ms) 안에 갱신해야 지령이 유지된다")]
        public float republishHz = 10f;
        [Tooltip("화면에 검증용 버튼을 그린다. 운영 빌드에서는 끈다")]
        public bool showDebugGui = true;

        ROSConnection _ros;
        readonly int[] _cmd = new int[DtBridgeConfig.CmdIdx.Length];
        float _next;
        string _lastSent = "—";

        void Start()
        {
            if (config == null) config = DtBridgeConfig.Instance;
            if (config == null) { enabled = false; return; }
            _ros = ROSConnection.GetOrCreateInstance();
            _ros.RegisterPublisher<RosInt32MultiArray>(config.commandTopic);
            Debug.Log($"[SafetyCommandSender] 발행 : {config.commandTopic}");
        }

        void Update()
        {
            if (republishHz <= 0f) return;
            if (Time.time < _next) return;
            _next = Time.time + 1f / republishHz;
            if (HasAny()) Publish();
        }

        bool HasAny()
        {
            foreach (var v in _cmd) if (v != 0) return true;
            return false;
        }

        void Publish()
        {
            var msg = new RosInt32MultiArray { data = (int[])_cmd.Clone() };
            _ros.Publish(config.commandTopic, msg);
        }

        // ---------------------------------------------------------- 공개 API
        public void Clear()
        {
            for (int i = 0; i < _cmd.Length; i++) _cmd[i] = 0;
            _lastSent = "해제";
            Publish();
        }

        /// <summary>정지 계열(normal/protective/emergency)은 서로 배타적으로 세운다.</summary>
        public void SetUrgent(int idx, string label)
        {
            _cmd[DtBridgeConfig.CmdIdx.Run] = 0;
            _cmd[DtBridgeConfig.CmdIdx.Hold] = 0;
            _cmd[DtBridgeConfig.CmdIdx.Stop] = 0;
            if (idx >= 0) _cmd[idx] = 1;
            _lastSent = label;
            Publish();
        }

        /// <summary>속도제한 계열(75/50/25 %)도 서로 배타적이다.</summary>
        public void SetSlowdown(int idx, string label)
        {
            _cmd[DtBridgeConfig.CmdIdx.SpeedDown1] = 0;
            _cmd[DtBridgeConfig.CmdIdx.SpeedDown2] = 0;
            _cmd[DtBridgeConfig.CmdIdx.SpeedDown3] = 0;
            if (idx >= 0) _cmd[idx] = 1;
            _lastSent = label;
            Publish();
        }

        public void SendNormal() => SetUrgent(DtBridgeConfig.CmdIdx.Run, "NORMAL");
        public void SendProtectiveStop() => SetUrgent(DtBridgeConfig.CmdIdx.Hold, "PROTECTIVE_STOP");
        public void SendEmergencyStop() => SetUrgent(DtBridgeConfig.CmdIdx.Stop, "EMERGENCY_STOP");
        public void SendReduced75() => SetSlowdown(DtBridgeConfig.CmdIdx.SpeedDown1, "REDUCED_SPEED_75");
        public void SendReduced50() => SetSlowdown(DtBridgeConfig.CmdIdx.SpeedDown2, "REDUCED_SPEED_50");
        public void SendReduced25() => SetSlowdown(DtBridgeConfig.CmdIdx.SpeedDown3, "REDUCED_SPEED_25");

        // ---------------------------------------------------------- 검증 GUI
        void OnGUI()
        {
            if (!showDebugGui) return;
            const int W = 250, H = 30;
            GUILayout.BeginArea(new Rect(12, 12, W, 330), GUI.skin.box);
            GUILayout.Label($"<b>제어 명령 → {config.robotId}</b>");
            GUILayout.Space(4);
            if (GUILayout.Button("정상 운전 · 전속", GUILayout.Height(H))) SendNormal();
            if (GUILayout.Button("보호정지 (전원 유지)", GUILayout.Height(H))) SendProtectiveStop();
            if (GUILayout.Button("비상정지", GUILayout.Height(H))) SendEmergencyStop();
            GUILayout.Space(6);
            if (GUILayout.Button("속도제한 75 %", GUILayout.Height(H))) SendReduced75();
            if (GUILayout.Button("속도제한 50 %", GUILayout.Height(H))) SendReduced50();
            if (GUILayout.Button("속도제한 25 %", GUILayout.Height(H))) SendReduced25();
            GUILayout.Space(6);
            if (GUILayout.Button("전체 해제", GUILayout.Height(H))) Clear();
            GUILayout.Space(4);
            GUILayout.Label($"마지막 발행 : {_lastSent}");
            GUILayout.EndArea();
        }
    }
}
