using System;
using System.Collections;
using Unity.VisualScripting;
using UnityEngine;

// 用于模拟四旋翼无人机的飞控程序，采用双环PID控制
public class Qcontorl : MonoBehaviour
{
    // X形四旋翼布局
    public GameObject rotor1;// 右前 顺时针
    public GameObject rotor2;// 左前 逆时针
    public GameObject rotor3;// 左后 顺时针
    public GameObject rotor4;// 右后 逆时针

    public float thisMass;// 机身重量

    // 初始高度
    public float setHeight = 5.0f;

    // 存储初始姿态
    private float setPitch;
    private float setRoll;
    private float setYaw;

    public float Kp, Ki, Kd;

    public float[] throttlePIDs = new float[5];
    public float[] attitudePIDs = new float[5];
    public float[] yawPIDs = new float[5];

    // PID计算器
    [SerializeField] public PID throttlePID;
    [SerializeField] public PID pitchPID;
    [SerializeField] public PID rollPID;
    [SerializeField] public PID yawPID;

    void Start()
    {
        LogRoot();
        // 设置初始目标高度，便于观察PID效果
        //初始化无人机姿态 欧拉角
        setPitch = 0;
        setRoll = 0;
        setYaw = 0;
    }

    void Update()
    {
        // 按键输入监测
        KeyScan();
    }

    private void FixedUpdate()
    {
        QControl();
    }


    // Quadrotor Control
    // 单环控制
    private void QControl()
    {
        // throttle 推力（转速）可视为输出到电机的电压
        throttlePID.SetPID(Kp, Ki, Kd, throttlePIDs[3], throttlePIDs[4]);
        throttlePID.RenewCalculData(setHeight, transform.position.y);
        float Thrrotle = throttlePID.Calcul();

        // Pitch 俯仰角
        pitchPID.SetPID(Kp,Ki,Kd, attitudePIDs[3], attitudePIDs[4]);
        float pitch = NormalizeAngle(transform.eulerAngles.x);
        pitchPID.RenewCalculData(setPitch, pitch);
        float pitch_cal = pitchPID.Calcul();

        // Roll 横滚角 
        rollPID.SetPID(Kp,Ki,Kd, attitudePIDs[3], attitudePIDs[4]);
        float roll = NormalizeAngle(transform.eulerAngles.z);
        rollPID.RenewCalculData(setRoll, roll);
        float roll_cal = rollPID.Calcul();

        // Yaw 偏航角
        yawPID.SetPID(Kp,Ki,Kd, yawPIDs[3], yawPIDs[4]);
        float yaw = NormalizeAngle1(transform.localEulerAngles.y);
        yawPID.RenewCalculData(setYaw, yaw);
        float yaw_cal = yawPID.Calcul();

        // 电机输出融合
        rotor1.GetComponent<QRotor>().throttle = Thrrotle - pitch_cal + roll_cal - yaw_cal;// 右前 顺
        rotor2.GetComponent<QRotor>().throttle = Thrrotle - pitch_cal - roll_cal + yaw_cal;// 左前 逆
        rotor3.GetComponent<QRotor>().throttle = Thrrotle + pitch_cal - roll_cal - yaw_cal;// 左后 顺
        rotor4.GetComponent<QRotor>().throttle = Thrrotle + pitch_cal + roll_cal + yaw_cal;// 右后 逆
    }

    // 角度归一化到[-180, 180]
    public float NormalizeAngle(float angle)
    {
        angle %= 360;
        if (angle > 180)
            return angle - 360;
        if (angle < -180)
            return angle + 360;
        return angle;
    }

    public float NormalizeAngle1(float angle)
    {
        if (angle > 180)
        {
            return angle - 360;
        }
        if (angle < -180)
        {
            return angle + 360;
        }
        return angle;
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
        if (Input.GetKey(KeyCode.Keypad8))
        {
            setHeight += 0.1f;
            Debug.Log($"setHigh:{setHeight}");
        }
        if (Input.GetKey(KeyCode.Keypad2))
        {
            setHeight -= 0.1f;
            Debug.Log($"setHigh:{setHeight}");
        }

        //左偏航
        if (Input.GetKey(KeyCode.Keypad4))
        {
            setYaw--;
            Debug.Log($"setYaw:{setYaw}");
        }
        //右偏航
        if (Input.GetKey(KeyCode.Keypad6))
        {
            setYaw++;
            Debug.Log($"setYaw:{setYaw}");
        }

        // vertical axis也包括A D 

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
    public float lastOutput, I, D;
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

    public PID(float Kp, float Ki, float Kd, float minLimit, float maxLimit)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.minLimit = minLimit;
        this.maxLimit = maxLimit;
    }

    // play模式调试PID
    public void SetPID(float Kp, float Ki, float Kd, float min, float max)
    {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.minLimit = min;
        this.maxLimit = max;
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
        I = Ki * integral;
        D = Kd * (now_error - previous_error);
        float Output = P + I + D;
        Output = Mathf.Clamp(Output, minLimit, maxLimit);
        previous_error = now_error;
        lastOutput = Output;
        return Output;
    }
}

public enum DroneState
{
    Hover,// 定高悬停 无人机尝试维持按下 `H` 时的 transform.position.y 高度,此时油门轴(W/S键)控制悬停高度
    Assist,// 辅助/姿态模式 油门依赖输入，接受姿态调控，无输入时能够自回正
    Manual,// 手动/特技模式 完全基于轴量控制
}