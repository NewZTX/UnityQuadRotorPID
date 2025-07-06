using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class QSensors : MonoBehaviour
{
    public Vector3 Acceleration { get; private set; }
    public Vector3 AngularVelocity { get; private set; }
    public float Pressure { get; private set; }
    public float Altitue { get; private set; }// true altitude

    public Gyroscope mGyroscope;
    public Accelerometer mAccelerometer;
    public Magnetometer mMagnetometer;
    public Barometer mBarometer;

   private void Start()
    {
        Rigidbody testRb = GetComponent<Rigidbody>();
        mGyroscope = new Gyroscope(testRb);
        mAccelerometer = new Accelerometer(testRb);
        mMagnetometer = new Magnetometer(testRb);
        mBarometer = new Barometer(testRb);
    }

    void FixedUpdate()
    {
        Acceleration = mAccelerometer.GetRawAcceleration();
        AngularVelocity = mGyroscope.GetRawAnglularVelocity();
        Pressure = mBarometer.GetPresure();
        Altitue = mBarometer.GetAltitude();// y变气压又变海拔......
    }
}

// TODO: 需要大改，全部使用基于世界空间坐标的速度和加速度计算，以及基于LocalRotation计算的角速度

// IMU
// 陀螺仪 三轴角速度计
public class Gyroscope
{
    public Rigidbody measureBody;

    public Gyroscope(Rigidbody measureBody)
    {
        this.measureBody = measureBody;
    }

    // `random.range(-1f,1f)` 用于模拟信号噪声
    // 应当先对传感器的输出进行评估，再决定添加的噪声的范围。

    public Vector3 GetRawAnglularVelocity()
    {
        return measureBody.angularVelocity * Mathf.Rad2Deg;
    }
}

// 三轴加速度计
public class Accelerometer
{
    public Rigidbody measureBody;
    private Vector3 lastVelocity;
    private Queue<Vector3> accBuffer = new(10);

    public Accelerometer(Rigidbody measureBody)
    {
        this.measureBody = measureBody;
        this.lastVelocity = measureBody.velocity;
    }

    // 必须在fixedUpdate调用,加速度计算使用的是Time.fixedDeltaTime
    public Vector3 GetRawAcceleration()
    {
        Vector3 currentVel = measureBody.velocity;
        Vector3 acc = (currentVel - lastVelocity) / Time.fixedDeltaTime;
        lastVelocity = currentVel;
        acc += Physics.gravity;
        // 滑动窗口滤波
        accBuffer.Enqueue(acc);
        if (accBuffer.Count > 10)
            accBuffer.Dequeue();

        // 计算平均值
        Vector3 sum = Vector3.zero;
        foreach (Vector3 sample in accBuffer)
        {
            sum += sample;
        }
        return sum / accBuffer.Count;
    }

    // 获取各轴加速度
    public float GetX() => GetRawAcceleration().x;
    public float GetY() => GetRawAcceleration().y;
    public float GetZ() => GetRawAcceleration().z;
}

//磁力计 快乐表
public class Magnetometer
{
    private const float earthMagField = 50.0f;
    public Rigidbody measureBody;

    public Magnetometer(Rigidbody measureBody)
    {
        this.measureBody = measureBody;
    }

    public Vector3 GetRawMagnetometer()
    {
        Quaternion rot = measureBody.rotation;
        Vector3 worldField = new(0.0f, 0.0f, earthMagField);
        Vector3 bodyField = rot * worldField;
        return bodyField;
    }
}

// 气压计 也是快乐表,不过帕斯卡看起来更有技术性,更专业
public class Barometer
{
    private const float seaLevelPresure = 1013.25f;
    private const float presureLapseRate = 0.12f;

    public Rigidbody measureBody;
    public Barometer(Rigidbody measureBody)
    {
        this.measureBody = measureBody;
    }

    public float GetPresure()
    {
        float height = measureBody.position.y;
        return seaLevelPresure - presureLapseRate * height;
    }

    public float GetAltitude()
    {
        float presure = GetPresure();
        return (seaLevelPresure - presure) / presureLapseRate;
    }
}