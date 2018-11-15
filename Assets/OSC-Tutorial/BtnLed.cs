using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class BtnLed : MonoBehaviour
{

    public OSCSender oscSender;

    public void StartLed()
    {
        oscSender.GetComponent<OSCSender>().triggerLed = true;
    }

    public void StopLed()
    {
        oscSender.GetComponent<OSCSender>().triggerLed = false;
    }


}