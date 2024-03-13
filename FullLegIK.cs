using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[DefaultExecutionOrder(3)]
public class FullLegIK : MonoBehaviour
{
    struct line
    {
        public Vector2 a;
        public Vector2 b;
    }
    struct DDDline
    {
        public Vector3 a;
        public Vector3 b;
    }
    struct Transf
    {
        public Vector3 position;
        public Quaternion rotation;
    }

    [Header("Main Parameters")]
    [Tooltip("Main Animator component")]
    [SerializeField] private Animator anim;
    [Tooltip("Main Transform that is being moved")]
    [SerializeField] private Transform ParentObject;
    [Tooltip("Hips bone on the skeleton")]
    [SerializeField] private Transform Hips;
    [Tooltip("Layers with which feet can interact")]
    [SerializeField] private LayerMask Mask;
    [Space]
    [Tooltip("Height of the raycast start position")]
    [SerializeField] private float RaycastHeight;
    [Tooltip("Distance from one hip to the center of the object")]
    [SerializeField] private float FeetOffset = 0.13f;
    [Tooltip("Enable automatic calculation of the required hip height while keeping the main object position intact")]
    [SerializeField] private bool OverideHipsHeight;
    [Tooltip("Distance betwen the hips and the foot on the lowest position")]
    [SerializeField] private float HipsHeight;
    [Tooltip("Spine bones that will be modified")]
    [SerializeField] private BoneModify[] SpineBones;
    [Tooltip("Enable arm movement")]
    [SerializeField] private bool SwayArms = true;
    [Tooltip("Multiplier for the arm swinging frequency")]
    [SerializeField, Range(0, 2)] private float ArmFrequencyMultilier = 1;
    [Tooltip("Multiplier for the arm swinging amplitude")]
    [SerializeField, Range(0, 0.3f)] private float ArmMultiplier = 0.15f;
    [Tooltip("Left arm rig")]
    [SerializeField] private ArmRig LeftArm;
    [Tooltip("Right arm rig")]
    [SerializeField] private ArmRig RightArm;

    [Space]
    [SerializeField] private Leg_Parameters LegParameters;

    [Space]
    [SerializeField] private Walking_Parameters WalkingParameters;

    [Header("Smoothing Parameters")]
    [Tooltip("Modifies how fast the speed lerps")]
    [SerializeField] private float SpeedLerp;
    [Tooltip("Modifies how fast the movement direction lerps")]
    [SerializeField] private float DirectionLerpSpeed;
    [Tooltip("Modifies how fast the walk frequency lerps")]
    [SerializeField] private float FrequencyLerpSpeed;
    [Tooltip("Modifies how fast the body goes to the requiered height")]
    [SerializeField] private float BobingLerpSpeed;
    [Tooltip("Modifies how fast the shoulders go to the requiered position")]
    [SerializeField] private float ShoulderLerpSpeed;

    private Transf LTransform;
    private Transf RTransform;

    private Transf LSettle;
    private Transf RSettle;

    private Transf LLast;
    private Transf RLast;

    private float Speed = 0;
    private float WalkFrequency = 2.4f;
    private float wantedF = 2.4f;
    private float addOnRot;

    Vector3 Lading;
    Vector3 Rading;
    Vector3 Lhint;
    Vector3 Rhint;

    Vector3 lastP = Vector3.zero;
    Vector2 movementDir;
    Vector2 LastDir;
    float TriggerAngle = 45;
    line DirLine;
    int lastStep = 1;
    bool ResetingP = false;
    Quaternion leftC;
    Quaternion RihtC;
    Vector3 DirLocal;

    Quaternion LastleftC;
    Quaternion LastRihtC;

    private float TimeSinceStep = 0;
    private float LstepT;
    private float RstepT;
    private float LastRot;
    private float rotSpeed;
    private float WrotSpeed;
    bool stopped = false;
    bool stoppedR = true;
    bool walking = false;
    Vector3 addLP;
    Vector3 addRP;
    Vector3 ALP;
    Vector3 ARP;

    Vector3 P1;
    Vector3 P2;
    Vector3 add;
    Vector3 LastCenter;
    Vector3 LastHips;
    Vector2 LFO;
    Vector2 RFO;
    float B;
    float Sp;
    float difference;
    float diff;
    float ArmTimer;
    bool back = true;
    float TurnF = 0;
    float LC;
    float RC;
    float timeLeft;
    int c = 0;
    Transf L;
    Transf R;
    Vector3 addUPL;
    Vector3 addUPR;
    Vector2 LastD;
    Vector2 LastDirL;
    Vector2 LastDirR;
    Vector3 lastPL;
    Vector3 lastPR;

    Vector3 WantedP1;
    Vector3 WantedP2;

    Vector3 PosL;
    Vector3 PosR;

    float FastSpeed = 0;

    [System.Serializable]
    public class Walking_Parameters
    {
        [Tooltip("Maximum distance that can be traveled in one step")]
        public float MaxStepDistance;
        [Tooltip("Minimum distance that can be traveled in one step")]
        public float MinStepDistance;
        [Tooltip("Multiplier for the distance of a step for the current speed")]
        public float DistanceMultiplier;
        [Tooltip("How much the hips should go down when moving fast")]
        [Range(0, 5)] public float ReduceHeightOnSprint;
        [Tooltip("How much the feet should go in direction of movement")]
        [Range(-0.5f, 0.5f)] public float DirectionAdd = 0.18f;
        [Tooltip("Enable to avoid feet steping on each other")]
        public bool AviodColision;
        [Tooltip("Distance for avoiding feet")]
        public float ColisionDistance;
        [Tooltip("Maximum angle betwen main object and feet before it gets corrected")]
        public float rotationError;
        [Tooltip("Curve that modifies the foot height while the foot is in the air")]
        public AnimationCurve FootRaise;
        [Tooltip("Foot raise height multiplier")]
        [Range(0, 0.5f)] public float FootRaiseHeight;
        [Tooltip("Enable to make the body lean in direction of movement")]
        public bool BodyLeaning;
        public float LeaningMultiplier;
        [Tooltip("Enable to make the body go up and down when walking")]
        public bool Bobing;
        public float BobingMultiplier;
        [Tooltip("Enable to make the shoulders go forward and backward when walking")]
        public bool ShoulderMove;
        public float ShoulderMultiplier;
        [Tooltip("Enable to rotate feet when they are in the air")]
        public bool RotateFeet;
        public float RotateFeetMultiplier;
        [Tooltip("Curve for feet rotation in the air")]
        public AnimationCurve FootRotateCurve;
        [Tooltip("Enable to step over any obsatcles in the way(not perfect, some clipping and twiching might happen)")]
        public bool ObstacleDetection;
        [Tooltip("Quality of the obstacle detection algorithm, anything betwen 5 and 15 is good")]
        public int ObstacleDetectionQuality;
    }

    [System.Serializable]
    public class Leg_Parameters
    {
        [Tooltip("IK Weight for feet position")]
        [Range(0f, 1f)] public float positionWeight;
        [Tooltip("IK Weight for feet rotation")]
        [Range(0f, 1f)] public float rotationWeight;
        [Tooltip("IK Weight for leg hint")]
        [Range(0f, 1f)] public float hintWeight;
        [Tooltip("Direction of the left leg hint relative to left foot rotated on global Y axis")]
        [Range(-180, 180)] public float LeftHintRotation = 0;
        [Tooltip("Direction of the left leg hint relative to right foot rotated on global Y axis")]
        [Range(-180, 180)] public float RightHintRotation = 0;
        [Tooltip("Hegiht offset for the feet")]
        [Range(-0.1f, 0.3f)] public float legHeight;
        [Tooltip("Maximum angle feet can be rotated to when standing on a steep surface")]
        public float MaxAngle;
        [Tooltip("Time it takes each foot to go to the default position when the character is idling")]
        public float ToDefaulPosTime;
        [Tooltip("Distance of the hint position from the foot")]
        public float HintDistance = 1;
        [Tooltip("Length of the foot, can be 0")]
        public float FootLength = 0.1f;

        [SerializeField] public LegPosition LeftLegOffset;
        [SerializeField] public LegPosition RightLegOffset;
    }

    [System.Serializable]
    public class LegPosition
    {
        [Tooltip("Foot position offset on the local X axis")]
        [Range(-0.5f, 0.5f)]
        public float positionX;
        [Tooltip("Foot position offset on the local Y axis")]
        [Range(-0.5f, 0.5f)]
        public float positionY;

        [Tooltip("Foot rotation offset on the global Y axis")]
        [Range(-90, 90)]
        public float YRotation;
    }

    [System.Serializable]
    public class BoneModify
    {
        [Tooltip("Spine bone")]
        public Transform bone;
        [Tooltip("How much the bone should be affected for leaning in direction of movement")]
        [Range(0, 100)]
        public float LeanPercentage;
        [Tooltip("How much the bone should be affected for shoulder rotation")]
        [Range(0, 100)]
        public float ShoulderPercentage;
    }

    [System.Serializable]
    public class ArmRig
    {
        public Transform Shoulder;
        [Tooltip("How much the shoulder should be affected for arm movement")]
        [Range(0, 100)]public float ShoulderPercentage;

        public Transform Arm;
        [Tooltip("How much the arm should be affected for arm movement")]
        [Range(0, 100)]public float ArmPercentage;

        public Transform Forearm;
        [Tooltip("How much the forearm should be affected for arm movement")]
        [Range(0, 100)]public float ForearmPercentage;

        [Tooltip("Changes how much the arm should go forward instead of backward")]
        [Range(0, 1)] public float ForwardAdd;
    }

    private void Update()
    {
        GetTime();
        float Lspeed = (1 - LstepT) / Mathf.Clamp(timeLeft, 0.01f, 1000);
        float Rspeed = (1 - RstepT) / Mathf.Clamp(timeLeft, 0.01f, 1000);
        LstepT = Mathf.Clamp01(LstepT);
        RstepT = Mathf.Clamp01(RstepT);
        LstepT += Mathf.Max(Lspeed, WalkFrequency) * Time.deltaTime;
        RstepT += Mathf.Max(Rspeed, WalkFrequency) * Time.deltaTime;
        DirLocal = ParentObject.rotation * v2T3(new Vector2(-movementDir.x, movementDir.y));
        DirLocal += ParentObject.forward * 0.03f;
        DirLocal.Normalize();

        Speed = Mathf.Round(Mathf.Lerp(Speed, CalculateObjectSpeed(), SpeedLerp * Time.deltaTime) * 10000) / 10000;
        FastSpeed = Mathf.Lerp(FastSpeed, CalculateObjectSpeed(), SpeedLerp * 10 * Time.deltaTime);
        Sp = Mathf.Round(Mathf.Lerp(Sp, CalculateObjectSpeed(), SpeedLerp * Time.deltaTime) * 10000) / 10000;
        Speed = Mathf.Clamp(Speed, 0, Mathf.Min(WalkingParameters.MaxStepDistance, FastSpeed));
        rotSpeed = Mathf.Lerp(rotSpeed, WrotSpeed, SpeedLerp * 20 * Time.deltaTime);
        diff = Mathf.Lerp(diff, difference, ShoulderLerpSpeed * Time.deltaTime);

        WalkFrequency = Mathf.Lerp(WalkFrequency, wantedF, FrequencyLerpSpeed * Time.deltaTime);
        movementDir = Vector2.Lerp(movementDir, GetMovementDir(), DirectionLerpSpeed * Time.deltaTime);
        TimeSinceStep += Time.deltaTime;

        if (CalculateObjectSpeed() <= 0.05f)
        {
            walking = false;
            if (!stopped && !ResetingP)
            {
                stopped = true;
                StartCoroutine(AlignLegs());
            }
        }
        else
        {
            stopped = false;
            walking = true;
        }

        if (!Rotating() && !stoppedR && !walking && !ResetingP)
        {
            stoppedR = true;
            StartCoroutine(AlignLegs());
        }
        else if (Rotating())
        {
            stoppedR = false;
        }

        if (ToWalk())
        {
            TakeAStep();
        }
        GetMovementLine();

        lastP = ParentObject.position;
        calculateRotSpeed();
    }
    [System.Obsolete]
    private void LateUpdate()
    {
        float M = 0;
        if (OverideHipsHeight)
        {
            M += (Hips.position.y - Mathf.Min(P1.y, P2.y)) - HipsHeight;
        }
        B = Mathf.Lerp(B, Vector2.Distance(v3To2(P1), v3To2(P2)) * WalkingParameters.BobingMultiplier / 10, Time.deltaTime * BobingLerpSpeed);
        if (WalkingParameters.Bobing)
        {
            M += B;
        }
        M += Sp * WalkingParameters.ReduceHeightOnSprint / 100;
        M += LegParameters.legHeight;

        add = Vector3.up * M;
        Hips.position -= add;
        if (WalkingParameters.BodyLeaning)
        {
            LeanInMovDir();
        }
        if (WalkingParameters.ShoulderMove)
        {
            ShoulderMovement();
        }
        if (SwayArms)
        {
            MoveArms();
        }
        float mul = Speed * WalkingParameters.DistanceMultiplier / 30;
        if (LC < 1)
        {
            float Dot = (Vector2.Dot(movementDir.normalized, LastD.normalized) + 2) / 3;
            Dot = Mathf.Clamp01(Dot);

            WantedP1 = lastPL;
            WantedP1 += v2T3(movementDir.normalized * mul * Dot);
            WantedP1 -= (LastCenter - GetCenterOfMass(LastHips));

            LTransform.position = Vector3.Lerp(LTransform.position, WantedP1, Time.deltaTime * 10);
            LastDirL = movementDir.normalized;


            LastCenter = GetCenterOfMass(LastHips);
        }
        else if (RC < 1)
        {
            float Dot = (Vector2.Dot(movementDir.normalized, LastD.normalized) + 2) / 3;
            Dot = Mathf.Clamp01(Dot);

            WantedP2 = lastPR;
            WantedP2 += v2T3(movementDir.normalized * mul * Dot);
            WantedP2 -= (LastCenter - GetCenterOfMass(LastHips));

            RTransform.position = Vector3.Lerp(RTransform.position, WantedP2, Time.deltaTime * 10);
            LastDirR = movementDir.normalized;

            LastCenter = GetCenterOfMass(LastHips);
        }
    }

    private void Awake()
    {
        LastRot = ParentObject.rotation.eulerAngles.y;
        lastP = ParentObject.position;
        TakeAStep();
        TakeAStep();

        LLast = LSettle;
        RLast = RSettle;
        P1 = LLast.position;
        P2 = RLast.position;
        TimeSinceStep = 3;
        RstepT = 10;
        LstepT = 10;
    }

    void OnAnimatorIK()
    {
        anim.SetIKPositionWeight(AvatarIKGoal.LeftFoot, LegParameters.positionWeight);
        anim.SetIKPositionWeight(AvatarIKGoal.RightFoot, LegParameters.positionWeight);
        anim.SetIKRotationWeight(AvatarIKGoal.LeftFoot, LegParameters.rotationWeight);
        anim.SetIKRotationWeight(AvatarIKGoal.RightFoot, LegParameters.rotationWeight);
        anim.SetIKHintPositionWeight(AvatarIKHint.RightKnee, LegParameters.hintWeight);
        anim.SetIKHintPositionWeight(AvatarIKHint.LeftKnee, LegParameters.hintWeight);

        anim.SetIKHintPosition(AvatarIKHint.LeftKnee, Lhint);
        anim.SetIKHintPosition(AvatarIKHint.RightKnee, Rhint);

        UpdateTransform();
        UpdateLegHints();
    }

    void UpdateTransform()
    {
        LC = LstepT + 0.001f;
        RC = RstepT + 0.001f;
        addLP = LTransform.rotation * Vector3.forward * LegParameters.LeftLegOffset.positionY + LTransform.rotation * Vector3.right * LegParameters.LeftLegOffset.positionX;
        addRP = RTransform.rotation * Vector3.forward * LegParameters.RightLegOffset.positionY + RTransform.rotation * Vector3.right * LegParameters.RightLegOffset.positionX;

        //LeftLeg
        Vector3 posL = LTransform.position + addLP;
        LSettle = RaycastLegs(posL, 0, false);

        P1 = Vector3.Lerp(LLast.position, LSettle.position, LC);
        P2 = Vector3.Lerp(RLast.position, RSettle.position, RC);

        Quaternion Lrot = Quaternion.identity;
        Quaternion Rrot = Quaternion.identity;

        if (WalkingParameters.RotateFeet)
        {
            Lrot = Quaternion.AngleAxis(WalkingParameters.FootRotateCurve.Evaluate(LstepT) * 90 * (Mathf.Clamp(DirLocal.z, -0.2f, 1) * Speed / WalkingParameters.MaxStepDistance) * WalkingParameters.RotateFeetMultiplier, LSettle.rotation * Vector3.right);
            Lrot *= Quaternion.AngleAxis(WalkingParameters.FootRotateCurve.Evaluate(LstepT) * 90 * (DirLocal.x * Speed / WalkingParameters.MaxStepDistance) * WalkingParameters.RotateFeetMultiplier / 3, LSettle.rotation * Vector3.forward);

            Rrot = Quaternion.AngleAxis(WalkingParameters.FootRotateCurve.Evaluate(RstepT) * 90 * (Mathf.Clamp(DirLocal.z, -0.2f, 1) * Speed / WalkingParameters.MaxStepDistance) * WalkingParameters.RotateFeetMultiplier, RSettle.rotation * Vector3.right);
            Rrot *= Quaternion.AngleAxis(WalkingParameters.FootRotateCurve.Evaluate(RstepT) * 90 * (DirLocal.x * Speed / WalkingParameters.MaxStepDistance) * WalkingParameters.RotateFeetMultiplier / 3, RSettle.rotation * Vector3.forward);
        }

        addUPL = Vector3.up * WalkingParameters.FootRaise.Evaluate(Mathf.Clamp01(LC)) * WalkingParameters.FootRaiseHeight * Vector2.Distance(v3To2(LSettle.position), v3To2(LLast.position));

        if (LFO.y < 1.1f && WalkingParameters.ObstacleDetection)
        {
            float y = CalculateY(LstepT, LFO.y) * LFO.x;
            addUPL += (Vector3.up * y) - Vector3.up * Mathf.Min(addUPL.y, y);
        }

        anim.SetIKPosition(AvatarIKGoal.LeftFoot, P1 + add
        + addUPL);

        anim.SetIKRotation(AvatarIKGoal.LeftFoot, Quaternion.Lerp(LLast.rotation.normalized, LSettle.rotation.normalized, LC + 0.02f)
        * Quaternion.AngleAxis(LegParameters.LeftLegOffset.YRotation, Vector3.up) * Quaternion.Lerp(LastleftC.normalized, leftC.normalized, LC + 0.02f) * Lrot);

        //RightLeg
        Vector3 posR = RTransform.position + addRP;
        RSettle = RaycastLegs(posR, 0, false);

        addUPR = Vector3.up * WalkingParameters.FootRaise.Evaluate(Mathf.Clamp01(RC)) * WalkingParameters.FootRaiseHeight * Vector2.Distance(v3To2(RSettle.position), v3To2(RLast.position));

        if (RFO.y < 1.1f && WalkingParameters.ObstacleDetection)
        {
            float y = CalculateY(RstepT, RFO.y) * RFO.x;
            addUPR += (Vector3.up * y) - Vector3.up * Mathf.Min(addUPR.y, y);
        }

        anim.SetIKPosition(AvatarIKGoal.RightFoot, P2 + add + addUPR);

        anim.SetIKRotation(AvatarIKGoal.RightFoot, Quaternion.Lerp(RLast.rotation.normalized, RSettle.rotation.normalized, RC + 0.02f)
        * Quaternion.AngleAxis(LegParameters.RightLegOffset.YRotation, Vector3.up) * Quaternion.Lerp(LastRihtC.normalized, RihtC.normalized, RC + 0.02f) * Rrot);


    }

    private Transf RaycastLegs(Vector3 raycastP, int foot, bool turn)
    {
        Transf T = new Transf();
        RaycastHit hit;
        if (Physics.Raycast(raycastP + Vector3.up * RaycastHeight, Vector3.down, out hit, Mathf.Infinity, Mask))
        {
            T.position = hit.point + hit.normal * LegParameters.legHeight;

            Vector3 slopeCorrected = Vector3.Cross(hit.normal, -Vector3.right);
            Quaternion footRotation = Quaternion.LookRotation(slopeCorrected, hit.normal);

            float a = Quaternion.Angle(Quaternion.identity, footRotation);

            if (Quaternion.Angle(Quaternion.identity, footRotation) < LegParameters.MaxAngle)
            {
                T.rotation = footRotation;
            }
            else
            {
                T.rotation = Quaternion.Lerp(Quaternion.identity, footRotation, LegParameters.MaxAngle / a);
            }
            if (foot == 1)
            {
                LastleftC = leftC;
                leftC = Quaternion.AngleAxis(ParentObject.rotation.eulerAngles.y, Vector3.up);
            }
            else if (foot == 2)
            {
                LastRihtC = RihtC;
                RihtC = Quaternion.AngleAxis(ParentObject.rotation.eulerAngles.y, Vector3.up);
            }
            if (turn)
            {
                T.rotation *= Quaternion.AngleAxis(ParentObject.rotation.eulerAngles.y, Vector3.up);
            }
        }
        return T;
    }

    int GetLegToMove()
    {
        Vector2 leftLegDir = new Vector2(Hips.position.x, Hips.position.z) - new Vector2(LTransform.position.x - FeetOffset, LTransform.position.z);
        Vector2 rightLegDir = new Vector2(Hips.position.x, Hips.position.z) - new Vector2(RTransform.position.x + FeetOffset, RTransform.position.z);

        bool T = Vector2.Angle(GetMovementDir(), LastDir) < TriggerAngle;

        bool D = Vector2.Dot(leftLegDir, movementDir.normalized) > Vector2.Dot(rightLegDir, movementDir.normalized);

        if (walking)
        {
            if (T)
            {
                if (D && lastStep != 1 || lastStep == 2)
                {
                    lastStep = 1;
                    return 1; // left
                }
                else
                {
                    lastStep = 2;
                    return 2; // right
                }
            }
            else
            {
                if (D)
                {
                    lastStep = 1;
                    return 1; // left
                }
                else
                {
                    lastStep = 2;
                    return 2; // right
                }
            }
        }
        else
        {
            if (Vector2.Distance(v3To2(LTransform.position), v3To2(Hips.position)) > Vector2.Distance(v3To2(RTransform.position), v3To2(Hips.position)))
            {
                lastStep = 1;
                return 1;
            }
            else
            {
                lastStep = 2;
                return 2;
            }
        }
    }

    private float CalculateObjectSpeed()
    {
        float SP = 0;

        SP = Vector2.Distance(new Vector2(lastP.x, lastP.z), new Vector2(ParentObject.position.x, ParentObject.position.z)) / Time.deltaTime;

        return SP;
    }

    private Vector2 GetMovementDir()
    {
        Vector3 dir = lastP - ParentObject.position;

        Vector2 D = -new Vector2(dir.x, dir.z);
        D.Normalize();

        return D;
    }

    private Vector3 GetCenterOfMass(Vector3 hips)
    {
        Vector3 P;
        if (walking)
        {
            Vector2 pos;
            pos = new Vector2(hips.x, hips.z) + movementDir.normalized * (Speed * WalkingParameters.DistanceMultiplier / 30);

            P = new Vector3(pos.x, Hips.position.y, pos.y);
        }
        else
        {
            P = Hips.position;
        }

        return P;
    }
    private void TakeAStep()
    {
        if (Vector2.Angle(GetMovementDir(), LastDir) > TriggerAngle)
        {
            c = 1;
            addOnRot = 0.5f;
        }
        else
        {
            addOnRot = 0;
        }

        if (c > 0)
        {
            Speed /= 10;
            c--;
        }

        LastCenter = GetCenterOfMass(Hips.position);
        LastHips = Hips.position;
        RLast = RSettle;
        LLast = LSettle;
        if (GetLegToMove() == 1)
        {
            LastDirL = movementDir.normalized;
            L = RaycastLegs(GetCenterOfMass(Hips.position), 1, true);
            LTransform.position = L.position;
            LTransform.rotation = L.rotation;
            Lading = v2T3(movementDir) * WalkingParameters.DirectionAdd * Speed;
            LTransform.position += Lading;

            if (WalkingParameters.AviodColision)
            {
                float dist = Vector2.Distance(v3To2(LTransform.position + addLP), v3To2(RTransform.position + addRP));

                if (dist < WalkingParameters.ColisionDistance)
                {
                    Vector2 dir = (v3To2(RTransform.position + addRP) - v3To2(LTransform.position + addLP)).normalized;
                    LTransform.position += v2T3(-dir * (WalkingParameters.ColisionDistance - dist));
                }
            }
            LstepT = 0;
            if (walking)
            {
                ALP = LTransform.rotation * Vector3.forward * LegParameters.LeftLegOffset.positionY * Mathf.Abs(DirLocal.z);
                LTransform.position -= ALP;
            }
            else
            {
                ALP = Vector3.zero;
            }
            if (WalkingParameters.ObstacleDetection)
            {
                Vector3 posL = LTransform.position + addLP;
                LSettle = RaycastLegs(posL, 0, false);
                LFO = GetOffsetHeight(LLast.position, LSettle.position, RaycastHeight, LegParameters.legHeight);
            }
            lastPL = LTransform.position - v2T3(movementDir.normalized * (Speed * WalkingParameters.DistanceMultiplier / 30));
        }
        else
        {
            LastDirR = movementDir.normalized;
            R = RaycastLegs(GetCenterOfMass(Hips.position), 2, true);
            RTransform.position = R.position;
            RTransform.rotation = R.rotation;
            Rading = v2T3(movementDir) * WalkingParameters.DirectionAdd * Speed;
            RTransform.position += Rading;

            if (WalkingParameters.AviodColision)
            {
                float dist = Vector2.Distance(v3To2(LTransform.position + addLP), v3To2(RTransform.position + addRP));

                if (dist < WalkingParameters.ColisionDistance)
                {
                    Vector2 dir = (v3To2(LTransform.position + addLP) - v3To2(RTransform.position + addRP)).normalized;
                    RTransform.position += v2T3(-dir * (WalkingParameters.ColisionDistance - dist));
                }
            }

            RstepT = 0;
            if (walking)
            {
                ARP = RTransform.rotation * Vector3.forward * LegParameters.RightLegOffset.positionY * Mathf.Abs(DirLocal.z);
                RTransform.position -= ARP;
            }
            else
            {
                ALP = Vector3.zero;
            }
            if (WalkingParameters.ObstacleDetection)
            {
                Vector3 posR = RTransform.position + addRP;
                RSettle = RaycastLegs(posR, 0, false);
                RFO = GetOffsetHeight(RLast.position, RSettle.position, RaycastHeight, LegParameters.legHeight);
            }
            lastPR = RTransform.position - v2T3(movementDir.normalized * (Speed * WalkingParameters.DistanceMultiplier / 30));
        }
        LastDir = GetMovementDir();
        LastD = movementDir.normalized;
        LastDirL = movementDir.normalized;
        LastDirR = movementDir.normalized;
        wantedF = 1 / TimeSinceStep;
        wantedF = Mathf.Clamp(wantedF, 0.5f, 10);
        TimeSinceStep = 0;
    }

    private bool ToWalk()
    {
        bool outside = false;
        bool LineS = false;

        if (walking)
        {
            LineS = PointDistanceToLine(DirLine, v3To2(Hips.position)) + WalkingParameters.MinStepDistance - ((WalkingParameters.MaxStepDistance / 4) * addOnRot) < Mathf.Min(PointDistanceToLine(DirLine, v3To2((LTransform.position - Lading) + ALP)), PointDistanceToLine(DirLine, v3To2((RTransform.position - Rading) + ARP)));
        }
        float angle = (Vector2.Angle(v3To2(LTransform.rotation * Vector3.forward), v3To2(ParentObject.forward)) + Vector2.Angle(v3To2(RTransform.rotation * Vector3.forward), v3To2(ParentObject.forward))) / 2;
        if (LineS || angle > WalkingParameters.rotationError * 2)
        {
            outside = true;
        }

        return outside;
    }

    private void GetMovementLine()
    {
        Vector2 dir = movementDir.normalized;
        DirLine.a = new Vector2(Hips.position.x, Hips.position.z) + dir * 3;
        DirLine.b = new Vector2(Hips.position.x, Hips.position.z) + dir * 3;

        DirLine.a += RotateVector(dir, 90, 0);
        DirLine.b += RotateVector(dir, -90, 0);
    }

    private float PointDistanceToLine(line line, Vector2 point)
    {
        Vector2 lineDirection = (line.a - line.b).normalized;
        Vector2 AtoPoint = point - line.a;
        float dot = Vector2.Dot(AtoPoint, lineDirection);
        Vector2 P2 = line.a + lineDirection * dot;

        return Vector2.Distance(point, P2);
    }

    private float PointDistanceOnLine(DDDline line, Vector3 point)
    {
        Vector3 lineDirection = (line.a - line.b).normalized;
        Vector3 AtoPoint = point - line.a;
        float dot = Vector3.Dot(AtoPoint, lineDirection);
        Vector3 P2 = line.a + lineDirection * dot;

        return Vector3.Distance(line.a, P2) / Vector3.Distance(line.a, line.b);
    }

    private Vector2 RotateVector(Vector2 vector, float angle, float angle1)
    {
        Quaternion rotation = Quaternion.Euler(angle1, 0f, angle);
        Vector2 rotatedVector = rotation * vector;
        return rotatedVector;
    }

    private Vector3 v2T3(Vector2 vector)
    {
        return new Vector3(vector.x, 0, vector.y);
    }
    private Vector2 v3To2(Vector3 vector)
    {
        return new Vector2(vector.x, vector.z);
    }

    private bool Rotating()
    {
        bool rotating = true;

        if (rotSpeed < 5)
        {
            rotating = false;
        }

        return rotating;
    }

    private void calculateRotSpeed()
    {
        WrotSpeed = Mathf.Abs(LastRot - ParentObject.rotation.eulerAngles.y) / Time.deltaTime;
        LastRot = ParentObject.rotation.eulerAngles.y;
    }

    IEnumerator AlignLegs()
    {
        Lading = Vector3.zero;
        Rading = Vector3.zero;
        ResetingP = true;
        if (walking && !stoppedR)
        {
            ResetingP = false;
            yield break;
        }
        yield return new WaitForSeconds(LegParameters.ToDefaulPosTime);
        if (walking && !stoppedR)
        {
            ResetingP = false;
            yield break;
        }
        TakeAStep();

        wantedF = 1 / LegParameters.ToDefaulPosTime;
        WalkFrequency = 1 / LegParameters.ToDefaulPosTime;
        yield return new WaitForSeconds(LegParameters.ToDefaulPosTime);
        if (walking && !stoppedR)
        {
            ResetingP = false;
            yield break;
        }
        TakeAStep();

        wantedF = 1 / LegParameters.ToDefaulPosTime;
        WalkFrequency = 1 / LegParameters.ToDefaulPosTime;
        ResetingP = false;
    }

    [System.Obsolete]
    private void LeanInMovDir()
    {
        TurnF = 0;
        foreach (BoneModify Bone in SpineBones)
        {
            TurnF += DirLocal.z * Bone.LeanPercentage / 150 * Sp * WalkingParameters.LeaningMultiplier / 6;
            Bone.bone.RotateAround(ParentObject.forward, (DirLocal.x * Speed / WalkingParameters.MaxStepDistance) * Bone.LeanPercentage / 150 * Sp * WalkingParameters.LeaningMultiplier / 6);
            Bone.bone.RotateAround(ParentObject.right, (DirLocal.z * Speed / WalkingParameters.MaxStepDistance) * Bone.LeanPercentage / 150 * Sp * WalkingParameters.LeaningMultiplier / 6);
        }
    }
    [System.Obsolete]
    private void ShoulderMovement()
    {
        difference = Vector3.Dot(ParentObject.forward, P1 - P2);
        difference -= Mathf.Abs(LegParameters.LeftLegOffset.positionY - LegParameters.RightLegOffset.positionY);

        foreach (BoneModify Bone in SpineBones)
        {
            Bone.bone.RotateAround(Vector3.up, diff * WalkingParameters.ShoulderMultiplier * Bone.ShoulderPercentage / 1000 * Speed / 1.5f);
        }
    }

    private void UpdateLegHints()
    {
        Lhint = P1;
        Lhint -= Vector3.up * Mathf.Abs(Hips.position.y + P1.y) / 2;

        Vector3 dir = v2T3(v3To2((anim.GetIKRotation(AvatarIKGoal.LeftFoot) * Vector3.forward + ParentObject.forward) / 2)) / 2 * LegParameters.HintDistance;
        dir = Quaternion.AngleAxis(LegParameters.LeftHintRotation, Vector3.up) * dir;
        Lhint += dir * 1.3f * WalkingParameters.MaxStepDistance;

        Rhint = P2;
        Rhint -= Vector3.up * Mathf.Abs(Hips.position.y + P2.y) / 2;

        dir = v2T3(v3To2((anim.GetIKRotation(AvatarIKGoal.RightFoot) * Vector3.forward + ParentObject.forward) / 2)) / 2 * LegParameters.HintDistance;
        dir = Quaternion.AngleAxis(LegParameters.RightHintRotation, Vector3.up) * dir;
        Rhint += dir * 1.3f * WalkingParameters.MaxStepDistance;
    }
    [System.Obsolete]
    private void MoveArms()
    {
        ArmTimer += Mathf.Min(WalkFrequency, Sp * 2) * ArmFrequencyMultilier * Time.deltaTime * (System.Convert.ToInt32(back) * 2 - 1);
        if (ArmTimer > 1)
        {
            back = false;
        }
        if (ArmTimer < -1)
        {
            back = true;
        }
        float v = Sp * ArmMultiplier * Mathf.Clamp(DirLocal.z + Mathf.Abs(DirLocal.x / 5), -1, 1);

        LeftArm.Shoulder.RotateAround(-transform.right, (ArmTimer + LeftArm.ForwardAdd * DirLocal.z) * LeftArm.ShoulderPercentage / 100 * v + TurnF);
        LeftArm.Arm.RotateAround(-transform.right, (ArmTimer + LeftArm.ForwardAdd * DirLocal.z) * LeftArm.ArmPercentage / 100 * v);
        LeftArm.Forearm.RotateAround(-transform.right, (ArmTimer + LeftArm.ForwardAdd) * LeftArm.ForearmPercentage / 100 * v);

        RightArm.Shoulder.RotateAround(-transform.right, -(ArmTimer - RightArm.ForwardAdd * DirLocal.z) * RightArm.ShoulderPercentage / 100 * v + TurnF);
        RightArm.Arm.RotateAround(-transform.right, -(ArmTimer - RightArm.ForwardAdd * DirLocal.z) * RightArm.ArmPercentage / 100 * v);
        RightArm.Forearm.RotateAround(-transform.right, -(ArmTimer - RightArm.ForwardAdd * DirLocal.z) * RightArm.ForearmPercentage / 100 * v);
    }

    private void GetTime()
    {
        float distance = Mathf.Min(PointDistanceToLine(DirLine, v3To2((LTransform.position - Lading) + ALP)), PointDistanceToLine(DirLine, v3To2((RTransform.position - Rading) + ARP))) - PointDistanceToLine(DirLine, v3To2(Hips.position)) + WalkingParameters.MinStepDistance - ((WalkingParameters.MaxStepDistance / 2) * addOnRot);
        timeLeft = Mathf.Abs(distance / Speed);
    }

    private Vector2 GetOffsetHeight(Vector3 StartPos, Vector3 EndPos, float CheckHeight, float FootHeight)
    {
        Vector2 height_Distance = new Vector2();
        RaycastHit hit = new RaycastHit();
        DDDline Dline = new DDDline();
        Vector3 Direction = EndPos - StartPos;
        if (Direction.magnitude == 0)
        {
            Direction = Vector3.one;
        }

        float CH = CheckHeight;
        Vector3 Center;
        Vector3 Size;
        float disance;
        float VerticalAdd = 0;
        Dline.a = StartPos;
        Dline.b = EndPos;
        height_Distance.y = 1.2f;

        for (int i = 0; i < WalkingParameters.ObstacleDetectionQuality; i++)
        {
            Center = StartPos + Vector3.up * CH / 2 + Vector3.up * (VerticalAdd - FootHeight);
            Size = new Vector3(0.1f, CH, 0.001f);
            disance = Vector3.Distance(StartPos, EndPos);

            if (Physics.BoxCast(Center, Size / 2, Direction, out hit, Quaternion.LookRotation(Direction), disance, Mask))
            {
                VerticalAdd += CH / 2;
                height_Distance.x = Center.y + CH / 2;
                height_Distance.x -= StartPos.y;
                height_Distance.y = PointDistanceOnLine(Dline, hit.point);
                height_Distance.x += FootHeight;
            }
            else
            {
                VerticalAdd -= CH / 2;
                height_Distance.x = Center.y - CH / 2;
                height_Distance.x -= StartPos.y;
                height_Distance.x += FootHeight;
            }

            VerticalAdd = Mathf.Max(VerticalAdd, 0);
            //DrawBox(Center, Direction, Size);

            CH /= 2;

        }
        height_Distance.x = Mathf.Max(height_Distance.x, 0);
        if (height_Distance.x >= 0.05f)
        {
            height_Distance.x += 0.2f * Speed / WalkingParameters.MaxStepDistance * WalkingParameters.RotateFeetMultiplier;
        }
        height_Distance.y -= LegParameters.FootLength;
        height_Distance.y = Mathf.Max(height_Distance.y, 0.01f);

        return height_Distance;
    }

    private float CalculateY(float v, float hitP)
    {
        hitP += 0.0001f;
        v += 0.0001f;
        float s = 1;
        float d = 1;


        if (v / hitP > 1)
        {
            s = Mathf.Clamp01((1 - v) / (1 - Mathf.Clamp(v, hitP, hitP + LegParameters.FootLength)));
        }
        else
        {
            d = Mathf.Clamp01(v / Mathf.Clamp(v, hitP, hitP + LegParameters.FootLength));
        }


        return (1 - Mathf.Abs(Mathf.Clamp(v, hitP, hitP + LegParameters.FootLength) - v)) * d * s;
    }

    void DrawBox(Vector3 Center, Vector3 Direction, Vector3 Size)
    {
        Vector3 forward = Direction;
        Vector3 up = Vector3.up * Size.y / 2;
        Vector3 right = Vector3.Cross(forward, up).normalized * Size.x / 2;

        Vector3 p1 = Center + forward + up + right;
        Vector3 p2 = Center + forward + up - right;
        Vector3 p3 = Center + forward - up + right;
        Vector3 p4 = Center + forward - up - right;
        Vector3 p5 = Center + up + right;
        Vector3 p6 = Center + up - right;
        Vector3 p7 = Center - up + right;
        Vector3 p8 = Center - up - right;

        Debug.DrawLine(p1, p2, Color.red, 1 / WalkFrequency);
        Debug.DrawLine(p2, p4, Color.red, 1 / WalkFrequency);
        Debug.DrawLine(p4, p3, Color.red, 1 / WalkFrequency);
        Debug.DrawLine(p3, p1, Color.red, 1 / WalkFrequency);

        Debug.DrawLine(p5, p6, Color.red, 1 / WalkFrequency);
        Debug.DrawLine(p6, p8, Color.red, 1 / WalkFrequency);
        Debug.DrawLine(p8, p7, Color.red, 1 / WalkFrequency);
        Debug.DrawLine(p7, p5, Color.red, 1 / WalkFrequency);

        Debug.DrawLine(p1, p5, Color.red, 1 / WalkFrequency);
        Debug.DrawLine(p2, p6, Color.red, 1 / WalkFrequency);
        Debug.DrawLine(p3, p7, Color.red, 1 / WalkFrequency);
        Debug.DrawLine(p4, p8, Color.red, 1 / WalkFrequency);
    }
}