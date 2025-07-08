using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using XCharts.Runtime;

public class ChartController : MonoBehaviour
{
    public int dataCount = 200;

    public LineChart heightChart;
    private float targetHeight, currentHeight;

    public LineChart pitchChart;
    private float targetPitch, currentPitch;

    public LineChart rollChart;
    private float targetRoll, currentRoll;

    public LineChart yawChart;
    private float targetYaw, currentYaw;

    public Qcontorl qcontorl;

    void Start()
    {
        SetChart(heightChart, "Height 高度");
        SetChart(pitchChart, "Pitch 俯仰");
        SetChart(rollChart, "Roll 横滚");
        SetChart(yawChart, "Yaw 偏航");
    }

    public void SetChart(LineChart chart, string name)
    {
        var title = chart.EnsureChartComponent<Title>();
        title.text = name;

        if (chart.GetSerie(0) != null)
        {
            chart.GetSerie(0).maxCache = dataCount;
            chart.GetSerie(0).maxShow = dataCount;
        }
        else if (chart.GetSerie(1) != null)
        {
            chart.GetSerie(1).maxCache = dataCount;
            chart.GetSerie(1).maxShow = dataCount;
        }
        else if (chart.GetSerie(1) == null)
        {
            var serie = chart.AddSerie<Line>(null, true, false);
            serie.maxCache = dataCount;
            serie.maxShow = dataCount;
        }
        chart.ClearData();
    }

    void FixedUpdate()
    {
        updateData();
        UpdateChartData(heightChart, targetHeight, currentHeight);
        UpdateChartData(pitchChart, targetPitch, currentPitch);
        UpdateChartData(rollChart, targetRoll, currentRoll);
        UpdateChartData(yawChart, targetYaw, currentYaw);
    }

    void updateData()
    {
        targetHeight = qcontorl.setHeight;
        currentHeight = qcontorl.transform.position.y;

        targetPitch = qcontorl.pitchPID.setPoint;
        currentPitch = qcontorl.NormalizeAngle(qcontorl.transform.eulerAngles.x);

        targetRoll = qcontorl.rollPID.setPoint;
        currentRoll = qcontorl.NormalizeAngle(qcontorl.transform.eulerAngles.z);

        targetYaw = qcontorl.yawPID.setPoint;
        currentYaw = qcontorl.NormalizeAngle(qcontorl.transform.eulerAngles.y);
    }

    public void UpdateChartData(LineChart chart, float value, float value1)
    {
        if (chart.GetSerie(0).dataCount >= dataCount)
        {
            chart.GetSerie(0).RemoveData(0);
            chart.GetSerie(0).AddData(value);
        }
        else
        {
            chart.GetSerie(0).AddData(value);
        }

        if (chart.GetSerie(1).dataCount >= dataCount)
        {
            chart.GetSerie(1).RemoveData(0);
            chart.GetSerie(1).AddData(value1);
        }
        else
        {
            chart.GetSerie(1).AddData(value1);
        }
    }
}