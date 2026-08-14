using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosInt32MultiArray = RosMessageTypes.Std.Int32MultiArrayMsg;

namespace SierraBase.RobotDT
{
    /// <summary>
    /// 메신저 ① 상태 수신부 — /robot/&lt;id&gt;/state (Int32MultiArray[7]) 구독.
    /// 정지 · 일시정지 · 감속 1/2/3 을 로봇 색상과 경광등으로 표현한다.
    /// </summary>
    public class RobotStateReceiver : MonoBehaviour
    {
        public enum SafetyLevel { Run, SlowDown1, SlowDown2, SlowDown3, Hold, Stop, Unknown }

        [Header("설정")]
        public DtBridgeConfig config;

        [Header("표시 대상")]
        [Tooltip("로봇 링크의 Renderer 들. 상태에 따라 색이 바뀐다")]
        public Renderer[] robotRenderers;
        [Tooltip("경광등 역할을 할 Light (없으면 무시)")]
        public Light beacon;
        [Tooltip("상태 텍스트를 표시할 TextMesh (없으면 무시)")]
        public TextMesh statusLabel;

        [Header("상태별 색")]
        public Color runColor = new Color(0.20f, 0.70f, 0.45f);
        public Color slow1Color = new Color(0.95f, 0.85f, 0.25f);
        public Color slow2Color = new Color(0.98f, 0.65f, 0.15f);
        public Color slow3Color = new Color(0.95f, 0.45f, 0.10f);
        public Color holdColor = new Color(0.85f, 0.25f, 0.20f);
        public Color stopColor = new Color(0.75f, 0.10f, 0.10f);
        public Color staleColor = new Color(0.55f, 0.55f, 0.58f);

        [Header("상태 (읽기 전용)")]
        [SerializeField] private SafetyLevel level = SafetyLevel.Unknown;
        [SerializeField] private int operationState;
        [SerializeField] private bool linkAlive;

        private float _lastRecv = -999f;
        private MaterialPropertyBlock _mpb;
        private static readonly int BaseColorId = Shader.PropertyToID("_BaseColor");
        private static readonly int ColorId = Shader.PropertyToID("_Color");

        public SafetyLevel Level => level;
        public bool LinkAlive => linkAlive;

        void Start()
        {
            if (config == null) config = DtBridgeConfig.Instance;
            if (config == null) { enabled = false; return; }
            _mpb = new MaterialPropertyBlock();
            ROSConnection.GetOrCreateInstance()
                .Subscribe<RosInt32MultiArray>(config.stateTopic, OnState);
            Debug.Log($"[RobotStateReceiver] 구독 : {config.stateTopic}");
        }

        void OnState(RosInt32MultiArray msg)
        {
            var d = msg.data;
            if (d == null || d.Length < DtBridgeConfig.StateIdx.Length) return;

            if (d[DtBridgeConfig.StateIdx.EmergencyStop] != 0) level = SafetyLevel.Stop;
            else if (d[DtBridgeConfig.StateIdx.Hold] != 0) level = SafetyLevel.Hold;
            else if (d[DtBridgeConfig.StateIdx.SpeedDown3] != 0) level = SafetyLevel.SlowDown3;
            else if (d[DtBridgeConfig.StateIdx.SpeedDown2] != 0) level = SafetyLevel.SlowDown2;
            else if (d[DtBridgeConfig.StateIdx.SpeedDown1] != 0) level = SafetyLevel.SlowDown1;
            else level = SafetyLevel.Run;

            operationState = d[DtBridgeConfig.StateIdx.OperationState];
            _lastRecv = Time.time;
        }

        void Update()
        {
            linkAlive = (Time.time - _lastRecv) <= config.staleTimeout;
            Color c = linkAlive ? ColorOf(level) : staleColor;

            if (robotRenderers != null)
            {
                foreach (var r in robotRenderers)
                {
                    if (r == null) continue;
                    r.GetPropertyBlock(_mpb);
                    _mpb.SetColor(BaseColorId, c);   // HDRP / URP
                    _mpb.SetColor(ColorId, c);       // Built-in
                    r.SetPropertyBlock(_mpb);
                }
            }

            if (beacon != null)
            {
                beacon.color = c;
                bool blink = level == SafetyLevel.Stop || level == SafetyLevel.Hold;
                beacon.intensity = !linkAlive ? 0.3f
                    : blink ? (Mathf.PingPong(Time.time * 4f, 1f) * 3f + 0.5f)
                    : 1.5f;
            }

            if (statusLabel != null)
            {
                statusLabel.text = linkAlive
                    ? $"{LabelOf(level)}   (op {operationState})"
                    : "통신 두절";
                statusLabel.color = c;
            }
        }

        Color ColorOf(SafetyLevel l) => l switch
        {
            SafetyLevel.Run => runColor,
            SafetyLevel.SlowDown1 => slow1Color,
            SafetyLevel.SlowDown2 => slow2Color,
            SafetyLevel.SlowDown3 => slow3Color,
            SafetyLevel.Hold => holdColor,
            SafetyLevel.Stop => stopColor,
            _ => staleColor,
        };

        static string LabelOf(SafetyLevel l) => l switch
        {
            SafetyLevel.Run => "운전",
            SafetyLevel.SlowDown1 => "감속 1 (25 %)",
            SafetyLevel.SlowDown2 => "감속 2 (50 %)",
            SafetyLevel.SlowDown3 => "감속 3 (75 %)",
            SafetyLevel.Hold => "일시정지",
            SafetyLevel.Stop => "비상정지",
            _ => "—",
        };
    }
}
