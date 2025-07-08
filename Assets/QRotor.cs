using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class QRotor : MonoBehaviour
{
    // 穿越机3D电机,扇叶反转也能产生和正转同样大小并反向的升力.

    public float throttle;// 转速 或视为输入电压
    [HideInInspector] public float Kf;//升力比例系数
    [HideInInspector] public float Km;//反扭矩比例系数 机体越小，该比例系数需要设置得越大

    [Header("螺旋桨")] public GameObject propeller;
    [Header("螺旋桨旋转轴")] public GameObject centerAxis;
    [Header("升力受力对象")] public GameObject stressedObjectForce;
    [Header("旋转受力对象")] public GameObject stressedObjectTorque;

    public bool clockwise = true;

    public float upForce;

    public float counterTorque;

    void Start()
    {
        Kf = 0.01f;
        Km = 0.01f;
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

        // 叶片转动
        propeller.transform.RotateAround(
            centerAxis.transform.position, centerAxis.transform.up, 10.0f*direction  * throttle * Time.deltaTime);

        // 产生升力
        // upForce = Kf * throttle * throttle; // 会导致推力恒为正
        float sign = Mathf.Sign(throttle);
        upForce = sign * Kf * throttle * throttle;
        stressedObjectForce.GetComponent<Rigidbody>().AddForce(stressedObjectForce.transform.up * upForce);

        // 产生反扭矩
        counterTorque = -sign * Km * throttle * throttle;
        // 加负号以实现 反 扭矩
        stressedObjectTorque.GetComponent<Rigidbody>().AddTorque(counterTorque * stressedObjectTorque.transform.up);
    }
    public void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position, transform.position + 0.1f * upForce * transform.up);
    }
}
