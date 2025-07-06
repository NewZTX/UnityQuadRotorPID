using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class QRotor : MonoBehaviour
{
    // 穿越机3D电机,扇叶反转也能产生和正转同样大小并反向的升力.

    public float throttle;// 转速
    [HideInInspector] public float Kf;//升力比例系数
    [HideInInspector] public float Km;//反扭矩比例系数

    [Header("螺旋桨")] public GameObject propeller;
    [Header("螺旋桨旋转轴")] public GameObject centerAxis;
    [Header("升力受力对象")] public GameObject stressedObjectForce;
    [Header("旋转受力对象")] public GameObject stressedObjectTorque;

    public bool clockwise = true;

    [HideInInspector] public float upForce;

    [HideInInspector] public float counterTorque;// 反扭距

    void Start()
    {
        Kf = 0.01f;
        Km = 0.005f;
    }

    void FixedUpdate()
    {
        float direction;
        if (clockwise)
        {
            direction = 1.0f;
        }
        else
        {
            direction = -1.0f;
        }

        propeller.transform.RotateAround(
            centerAxis.transform.position, centerAxis.transform.up, direction * 50f * throttle * Time.deltaTime);

        // upForce = Kf * throttle * throttle; // 会导致推力恒为正
        float sign = Mathf.Sign(throttle);
        upForce = sign * Kf * throttle * throttle;
        stressedObjectForce.GetComponent<Rigidbody>().AddForce(stressedObjectForce.transform.up * upForce);

        counterTorque = sign * Km * throttle * throttle;
        stressedObjectTorque.GetComponent<Rigidbody>().AddRelativeTorque(counterTorque * direction * stressedObjectTorque.transform.up);
    }
}
