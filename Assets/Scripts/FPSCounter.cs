using TMPro;
using UnityEngine;

public class FPSCounter : MonoBehaviour
{
    public float FPSMeasurePeriod = 1f;

    private TextMeshProUGUI textField;

    private float lastFPSMeasureTimeStamp;

    private int avgFPS = -1;

    private int frameCounter = 0;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        textField = GetComponent<TextMeshProUGUI>();

        lastFPSMeasureTimeStamp = Time.realtimeSinceStartup;
    }

    // Update is called once per frame
    void Update()
    {
        frameCounter++;

        float now = Time.realtimeSinceStartup;
        float deltaTime = now - lastFPSMeasureTimeStamp;

        if (deltaTime > FPSMeasurePeriod)
        {
            avgFPS = Mathf.RoundToInt(frameCounter / deltaTime);
            lastFPSMeasureTimeStamp = now;
            frameCounter = 0;
        }

        float instantFPS = Mathf.RoundToInt(1.0f / Time.smoothDeltaTime);

        if (avgFPS == -1)
        {
            textField.text = $"FPS: instant={instantFPS}, avg=n/a";
        }
        else
        {
            textField.text = $"FPS: instant={instantFPS}, avg={avgFPS}";
        }
    }
}
