using System.Collections;
using UnityEngine;

public class FollowWaypoints : MonoBehaviour
{
    public Transform[] waypoints;
    private int currentWaypointIndex = 0;
    public float speed = 35.0f;
    private bool isMoving = true; // 추가: 이동 여부를 나타내는 변수

    void Update()
    {
        if (waypoints.Length == 0 || !isMoving) return;

        Transform targetWaypoint = waypoints[currentWaypointIndex];
        Vector3 direction = targetWaypoint.position - transform.position;
        transform.position += direction.normalized * speed * Time.deltaTime;

        if (Vector3.Distance(transform.position, targetWaypoint.position) < 0.1f)
        {
            // 첫 번째 웨이포인트 도착 시 대기
            if (currentWaypointIndex == 0)
            {
                Debug.Log("wait before start");
                StartCoroutine(WaitAndMoveToNextWaypoint(5.0f));
            }
            else if (currentWaypointIndex == 3) {
                Debug.Log("adjusting");
                StartCoroutine(WaitAndMoveToNextWaypoint(0.5f));
            }
            else if (currentWaypointIndex == 4) {
                Debug.Log("arrived");
                StartCoroutine(WaitAndMoveToNextWaypoint(60.0f));
            }
            else
            {
                currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length;
            }
        }
    }

    IEnumerator WaitAndMoveToNextWaypoint(float waitTime)
    {
        isMoving = false;
        yield return new WaitForSeconds(waitTime);
        currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length;
        isMoving = true;
    }
}

