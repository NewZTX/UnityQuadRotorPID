using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public GameObject target;
    public Vector3 offSet;
    // Start is called before the first frame update
    void Start()
    {
        offSet = target.transform.position - transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        gameObject.transform.position = target.transform.position - offSet;
    }
}
