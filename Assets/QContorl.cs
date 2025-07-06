using System;
using UnityEngine;

// 用于模拟四旋翼无人机的飞控程序，采用双环PID控制
public class Qcontorl : MonoBehaviour
{
    // X形四旋翼布局
    public GameObject rotor1;// 右前 顺时针
    public float motor1Mass;// 右前电机重量
    public GameObject rotor2;// 左前 逆时针
    public float motor2Mass;// 左前电机重量
    public GameObject rotor3;// 左后 顺时针
    public float motor3Mass;// 左前电机重量
    public GameObject rotor4;// 右后 逆时针
    public float motor4Mass;// 右前电机重量

    // 转动惯量 无人机在不同转动轴向上角速度的空气阻力减速效率不同
    // y轴阻力更小
    public Vector3 inertiaMoment;// (0.2,0.1,0.2)
    public QSensors mySensors;// 传感器
    public float thisMass;// 机身重量

    // 初始高度
    public float setHigh = 5.0f;

    // 存储初始姿态
    private float setPitch;
    private float setRoll;
    private float setYaw;

    // PID计算器
    [SerializeField] private PID throttlePID;
    [SerializeField] private PID pitchPID;
    [SerializeField] private PID rollPID;
    [SerializeField] private PID yawPID;
    [SerializeField] private PID pitchRatePID;
    [SerializeField] private PID rollRatePID;
    [SerializeField] private PID yawRatePID;

    [Header("油门PID")]
    [Range(0, 20)] public float Kp;// 2.5
    [Range(0, 1)] public float Ki;// 0.02
    [Range(0, 20)] public float Kd;// 12

    [Header("油门PID输出限制")]
    public float PIDupLimit;
    public float PIDdownLimit;

    [Header("姿态PID")]
    [Range(0, 20)] public float Kp1;
    [Range(0, 1)] public float Ki1;
    [Range(0, 20)] public float Kd1;

    [Header("姿态PID输出限制")]
    public float PIDupLimit1;
    public float PIDdownLimit1;

    void Start()
    {
        LogRoot();
        // pitchFilter = new KalManFilter(0.1f, 0.1f, 0, 0.1f, 0.5f);
        // rollFilter = new KalManFilter(0.1f, 0.1f, 0, 0.1f, 0.5f);
        // yawFilter = new KalManFilter(0.1f, 0.1f, 0, 0.1f, 0.5f);

        setHigh = this.transform.position.y;
        //初始化欧拉角
        setPitch = 0;
        setRoll = 0;
        setYaw = 0;

        throttlePID = new PID(Kp, Ki, Kd, PIDdownLimit, PIDupLimit);
        pitchPID = new PID(Kp1, Ki1, Kd1, PIDdownLimit1, PIDupLimit1);
        rollPID = new PID(Kp1, Ki1, Kd1, PIDdownLimit1, PIDupLimit1);
        yawPID = new PID(Kp1, Ki1, Kd1, PIDdownLimit1, PIDupLimit1);

        gameObject.GetComponent<Rigidbody>().mass = thisMass;
        rotor1.GetComponent<Rigidbody>().mass = motor1Mass;
        rotor2.GetComponent<Rigidbody>().mass = motor2Mass;
        rotor3.GetComponent<Rigidbody>().mass = motor3Mass;
        rotor4.GetComponent<Rigidbody>().mass = motor4Mass;
    }

    void Update()
    {
        // 按键输入监测
        KeyScan();
    }

    private void FixedUpdate()
    {
        // QControl();
        DualLoopPIDControl();
    }
    
    /*
    // Quadrotor Control
    // 单环控制
    private void QControl()
    {
        // throttle 推力（转速）
        throttlePID.SetPID(Kp, Ki, Kd);
        throttlePID.RenewCalculData(setHigh, mySensors.transform.position.y);
        float Thrrotle = throttlePID.Calcul("throttle");
        // Debug.Log($"throrttlePID>>>{throttlePID.Kp}>>>{throttlePID.Ki}>>>{throttlePID.Kd} \n" + $"ThrottleDiff>>>{throttlePID.now_error}>>>ThrottleValue:{Thrrotle}");

        // Pitch 俯仰角
        pitchPID.SetPID(Kp1, Ki1, Kd1);
        float pitch = mySensors.transform.eulerAngles.x;
        if (pitch > 180.0f)
            pitch -= 360.0f;
        else if (pitch < -180.0f)
            pitch -= 360.0f;
        pitchPID.RenewCalculData(setPitch, pitch);
        float pitch_cal = pitchPID.Calcul("Pitch");

        // Roll 横滚角
        rollPID.SetPID(Kp1, Ki1, Kd1);
        float roll = mySensors.transform.eulerAngles.z;
        if (roll > 180.0f)
            roll -= 360.0f;
        else if (roll < -180.0f)
            roll += 360.0f;
        rollPID.RenewCalculData(setRoll, roll);
        float roll_cal = rollPID.Calcul("Roll");

        // Yaw 偏航角
        yawPID.SetPID(Kp1, Ki1, Kd1);
        float yaw = mySensors.transform.localEulerAngles.y;
        if (yaw > 180.0f)
            yaw -= 360.0f;
        else if (yaw < -180.0f)
            yaw += 360.0f;
        yawPID.RenewCalculData(setYaw, yaw);
        float yaw_cal = yawPID.Calcul("Yaw");

        // 电机输出融合
        rotor1.GetComponent<QRotor>().throttle = Thrrotle - pitch_cal + roll_cal - yaw_cal;// 右前 顺
        rotor2.GetComponent<QRotor>().throttle = Thrrotle - pitch_cal - roll_cal + yaw_cal;// 左前 逆
        rotor3.GetComponent<QRotor>().throttle = Thrrotle + pitch_cal - roll_cal - yaw_cal;// 左后 顺
        rotor4.GetComponent<QRotor>().throttle = Thrrotle + pitch_cal + roll_cal + yaw_cal;// 右后 逆
    }
    */

    // 串级PID控制
    // 用于替换单环控制
    public void DualLoopPIDControl()
    {
        // 1. 计算油门基准值（高度控制）
        throttlePID.SetPID(Kp, Ki, Kd);
        throttlePID.RenewCalculData(setHigh, mySensors.transform.position.y);
        float baseThrottle = throttlePID.Calcul("油门");
        // 由于无人机处于重力环境，无人机低于setheight的时间更长，所以integral会持续累积

        // 2. 外环PID（角度环）→ 输出目标角速度
        Vector3 targetAngularVelocity = Vector3.zero;

        // Pitch外环
        float currentPitch = NormalizeAngle(mySensors.transform.eulerAngles.x);
        pitchPID.SetPID(Kp1, Ki1, Kd1);
        pitchPID.RenewCalculData(setPitch, currentPitch);
        targetAngularVelocity.x = pitchPID.Calcul("外环俯仰角");

        // Roll外环
        float currentRoll = NormalizeAngle(mySensors.transform.eulerAngles.z);
        rollPID.SetPID(Kp1, Ki1, Kd1);
        rollPID.RenewCalculData(setRoll, currentRoll);
        targetAngularVelocity.z = rollPID.Calcul("外环横滚角");

        // Yaw外环
        float currentYaw = NormalizeAngle(mySensors.transform.localEulerAngles.y);
        yawPID.SetPID(Kp1, Ki1, Kd1);
        yawPID.RenewCalculData(setYaw, currentYaw);
        targetAngularVelocity.y = yawPID.Calcul("外环航向角");

        // 3. 内环PID（角速度环）→ 输出电机控制量
        Vector3 angularControl = Vector3.zero;
        Vector3 currentAngularVelocity = mySensors.AngularVelocity;

        // 初始化内环PID
        pitchRatePID ??= new PID(Kp1, Ki1, Kd1, PIDdownLimit1, PIDupLimit1);
        rollRatePID ??= new PID(Kp1, Ki1, Kd1, PIDdownLimit1, PIDupLimit1);
        yawRatePID ??= new PID(Kp1, Ki1, Kd1, PIDdownLimit1, PIDupLimit1);

        // Pitch内环
        pitchRatePID.RenewCalculData(targetAngularVelocity.x, currentAngularVelocity.x);
        angularControl.x = pitchRatePID.Calcul("内环俯仰角");

        // Roll内环
        rollRatePID.RenewCalculData(targetAngularVelocity.z, currentAngularVelocity.z);
        angularControl.z = rollRatePID.Calcul("内环俯仰角");

        // Yaw内环
        yawRatePID.RenewCalculData(targetAngularVelocity.y, currentAngularVelocity.y);
        angularControl.y = yawRatePID.Calcul("内环航向角");

        // 4. 电机指令融合
        rotor1.GetComponent<QRotor>().throttle = baseThrottle - angularControl.x + angularControl.z - angularControl.y;
        rotor2.GetComponent<QRotor>().throttle = baseThrottle - angularControl.x - angularControl.z + angularControl.y;
        rotor3.GetComponent<QRotor>().throttle = baseThrottle + angularControl.x - angularControl.z - angularControl.y;
        rotor4.GetComponent<QRotor>().throttle = baseThrottle + angularControl.x + angularControl.z + angularControl.y;
    }

    // 角度归一化到[-180, 180]
    private float NormalizeAngle(float angle)
    {
        angle %= 360;
        return angle > 180 ? angle - 360 : angle < -180 ? angle + 360 : angle;
    }

    // 键盘操控
    // W S 控制目标悬停高度
    // up down 控制俯仰角
    // left right 控制横滚角
    // A D 控制航向角
    private void KeyScan()
    {
        if (Input.GetKey(KeyCode.R))
        {
            ResetRoot();
        }

        //设置悬停高度大小
        if (Input.GetKey(KeyCode.W))
        {
            setHigh += 0.5f;
            Debug.Log($"setHigh:{setHigh}");
        }
        if (Input.GetKey(KeyCode.S))
        {
            setHigh -= 0.5f;
            Debug.Log($"setHigh:{setHigh}");
        }

        //左偏航
        if (Input.GetKey(KeyCode.A))
        {
            setYaw--;
            Debug.Log($"setYaw:{setYaw}");
        }
        //右偏航
        if (Input.GetKey(KeyCode.D))
        {
            setYaw++;
            Debug.Log($"setYaw:{setYaw}");
        }

        // 俯仰角最大最小值
        float pitchLimit = 30;
        // 向前向后
        setPitch = Input.GetAxis("Vertical") * pitchLimit;// 无输入时vertical和setpitch会自动归零
        // 横滚角最大最小值
        float rollLimit = 30;
        // 向左向右
        setRoll = -Input.GetAxis("Horizontal") * rollLimit;
    }

    // 手柄控制
    private void JoystickControl()
    {
        // TODO: 获取xbox360手柄输入，根据杆量设定roll yaw pitch
    }

    // 重置，用于快速测试PID参数
    private Vector3[] startPosition = new Vector3[10];
    private Quaternion[] startRotation = new Quaternion[10];

    private void LogRoot()
    {
        startPosition[0] = transform.position;
        startRotation[0] = transform.rotation;

        startPosition[1] = rotor1.transform.position;
        startRotation[1] = rotor1.transform.rotation;
        startPosition[2] = rotor2.transform.position;
        startRotation[2] = rotor2.transform.rotation;
        startPosition[3] = rotor3.transform.position;
        startRotation[3] = rotor3.transform.rotation;
        startPosition[4] = rotor4.transform.position;
        startRotation[4] = rotor4.transform.rotation;
    }

    private void ResetRoot()
    {
        transform.SetPositionAndRotation(startPosition[0], startRotation[0]);
        gameObject.GetComponent<Rigidbody>().velocity = Vector3.zero;

        rotor1.transform.SetPositionAndRotation(startPosition[1], startRotation[1]);
        rotor1.GetComponent<Rigidbody>().velocity = Vector3.zero;
        rotor2.transform.SetPositionAndRotation(startPosition[2], startRotation[2]);
        rotor2.GetComponent<Rigidbody>().velocity = Vector3.zero;
        rotor3.transform.SetPositionAndRotation(startPosition[3], startRotation[3]);
        rotor3.GetComponent<Rigidbody>().velocity = Vector3.zero;
        rotor4.transform.SetPositionAndRotation(startPosition[4], startRotation[4]);
        rotor4.GetComponent<Rigidbody>().velocity = Vector3.zero;
    }
}

//PID计算器 带有输出值clamp
[Serializable]
public class PID
{
    public float setPoint;
    public float measuredPoint;
    public float integral;//积分
    private float previous_error = 0f;
    public float now_error;

    public float Kp;
    public float Ki;
    public float Kd;
    private float minLimit;
    private float maxLimit;

    //构造方法，用来初始化对象
    public PID(float Kp, float Ki, float Kd, float minLimit, float maxLimit)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.minLimit = minLimit;
        this.maxLimit = maxLimit;
    }

    // play模式调试PID
    public void SetPID(float Kp, float Ki, float Kd)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public void RenewCalculData(float setPoint, float measuredPoint)
    {
        this.setPoint = setPoint;
        this.measuredPoint = measuredPoint;
    }

    public float Calcul()
    {
        now_error = setPoint - measuredPoint;
        float P = Kp * now_error;
        integral += now_error;
        float I = Ki * integral;
        float D = Kd * (now_error - previous_error);
        float Output = P + I + D;
        Output = Mathf.Clamp(Output, minLimit, maxLimit);
        previous_error = now_error;
        return Output;
    }

    // 测试用

    public float Calcul(string objectname)
    {
        float now_error = setPoint - measuredPoint;
        float P = Kp * now_error;
        integral += now_error;
        float I = Ki * integral;
        float D = Kd * (now_error - previous_error);
        float Output = P + I + D;
        Debug.Log($"{objectname} Output:{Output} ");
        Output = Mathf.Clamp(Output, minLimit, maxLimit);
        previous_error = now_error;
        return Output;
    }
}

//卡尔曼滤波器 用于稳定传感器输入
public class KalManFilter
{
    //基本参数
    public float LastP;
    public float NowP;
    public float Output;
    public float Kg;
    public float Q;
    public float R;

    public KalManFilter(float LastP, float NowP, float Kg, float Q, float R)
    {
        this.LastP = LastP;
        this.NowP = NowP;
        this.Kg = Kg;
        this.Q = Q;
        this.R = R;
    }

    public float CalOut(float Input)
    {
        NowP = LastP + Q;
        Kg = NowP / (NowP + R);
        Output = Output + Kg * (Input - Output);
        LastP = (1 - Kg) * NowP;
        return Output;
    }
}

public enum DroneState
{
    Hover,// 定高悬停 无人机尝试维持按下 `H` 时的 transform.position.y 高度,此时油门轴(W/S键)控制悬停高度
    Assist,// 辅助/姿态模式 油门依赖输入，接受姿态调控，无输入时能够自回正
    Manual,// 手动/特技模式 完全基于轴量控制
}