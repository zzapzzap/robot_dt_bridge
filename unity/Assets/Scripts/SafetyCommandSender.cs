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
    /// XDI(정지) · XAG(감속) 지령을 덮어쓰지 못한다 — 안전상 의도된 설계다.
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

        /// <summary>긴급 계열(run/hold/stop)은 서로 배타적으로 세운다.</summary>
        public void SetUrgent(int idx, string label)
        {
            _cmd[DtBridgeConfig.CmdIdx.Run] = 0;
            _cmd[DtBridgeConfig.CmdIdx.Hold] = 0;
            _cmd[DtBridgeConfig.CmdIdx.Stop] = 0;
            if (idx >= 0) _cmd[idx] = 1;
            _lastSent = label;
            Publish();
        }

        /// <summary>감속 계열(1/2/3)도 서로 배타적이다.</summary>
        public void SetSlowdown(int idx, string label)
        {
            _cmd[DtBridgeConfig.CmdIdx.SpeedDown1] = 0;
            _cmd[DtBridgeConfig.CmdIdx.SpeedDown2] = 0;
            _cmd[DtBridgeConfig.CmdIdx.SpeedDown3] = 0;
            if (idx >= 0) _cmd[idx] = 1;
            _lastSent = label;
            Publish();
        }

        public void SendRun() => SetUrgent(DtBridgeConfig.CmdIdx.Run, "운전");
        public void SendHold() => SetUrgent(DtBridgeConfig.CmdIdx.Hold, "일시정지");
        public void SendStop() => SetUrgent(DtBridgeConfig.CmdIdx.Stop, "비상정지");
        public void SendSlow1() => SetSlowdown(DtBridgeConfig.CmdIdx.SpeedDown1, "감속 1 (25 % 감속)");
        public void SendSlow2() => SetSlowdown(DtBridgeConfig.CmdIdx.SpeedDown2, "감속 2 (50 % 감속)");
        public void SendSlow3() => SetSlowdown(DtBridgeConfig.CmdIdx.SpeedDown3, "감속 3 (75 % 감속)");

        // ---------------------------------------------------------- 검증 GUI
        void OnGUI()
        {
            if (!showDebugGui) return;
            const int W = 250, H = 30;
            GUILayout.BeginArea(new Rect(12, 12, W, 330), GUI.skin.box);
            GUILayout.Label($"<b>제어 명령 → {config.robotId}</b>");
            GUILayout.Space(4);
            if (GUILayout.Button("운전", GUILayout.Height(H))) SendRun();
            if (GUILayout.Button("일시정지 (Hold)", GUILayout.Height(H))) SendHold();
            if (GUILayout.Button("비상정지 (Stop)", GUILayout.Height(H))) SendStop();
            GUILayout.Space(6);
            if (GUILayout.Button("감속 1 · 25 % 감속", GUILayout.Height(H))) SendSlow1();
            if (GUILayout.Button("감속 2 · 50 % 감속", GUILayout.Height(H))) SendSlow2();
            if (GUILayout.Button("감속 3 · 75 % 감속", GUILayout.Height(H))) SendSlow3();
            GUILayout.Space(6);
            if (GUILayout.Button("전체 해제", GUILayout.Height(H))) Clear();
            GUILayout.Space(4);
            GUILayout.Label($"마지막 발행 : {_lastSent}");
            GUILayout.EndArea();
        }
    }
}
