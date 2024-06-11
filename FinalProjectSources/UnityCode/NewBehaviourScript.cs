using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// 초기화 -> 물리 -> 게임로직 -> 해체
public class NewBehaviourScript : MonoBehaviour
{
    // 초기화
    void Awake() { // 최초 실행, 한 번만 실행
        Debug.Log("Power ON");
    }

    // 활성화
    void OnEnable() { // 오브젝트 활성화 시
        Debug.Log("Log In");
    }
    void start() { // 업데이트 시작 직전, 한 번만 실행
        Debug.Log("Ready");
    }

    // 물리연산
    void FixedUpdate() {
        Debug.Log("Move");
    }
    
    // 게임로직
    void Update() {
        Debug.Log("Grab");
    }
    void LateUpdate() {
        Debug.Log("Release");
    }

    // 비활성화
    void OnDisable() {
        Debug.Log("Log Out");
    }

    // 해체
    void OnDestroy() { // 오브젝트 삭제 시
        Debug.Log("Power OFF");
    }
}
