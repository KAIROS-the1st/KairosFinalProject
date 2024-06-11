using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NewBehaviourScript1 : MonoBehaviour
{
    void Update() {
        // 현재 위치를 확인합니다.
        Vector3 currentPosition = transform.position;

        // x좌표가 15보다 작을 때만 이동합니다.
        if (currentPosition.x < 95) {
            Vector3 vec = new Vector3(3, 0 , 0);
            transform.Translate(vec * Time.deltaTime);
        }
    }
}
