using System.Collections;
using UnityEngine;

public class myAgvController : MonoBehaviour
{
    public Transform[] waypoints;  // AGV가 이동할 웨이포인트
    public Transform[] personWaypoints;  // 사람이 이동할 웨이포인트
    public GameObject elevator;    // 엘리베이터 오브젝트 참조
    public float speed = 1.0f;     // 이동 속도 조정
    private int currentWaypointIndex = 0;
    private bool isMoving = true;
    private bool isOnElevator = false;
    private bool hasPickedUpPerson = false;  // 사람이 따라가게 했는지 여부

    void Update()
    {
        if (waypoints.Length == 0 || !isMoving) return;

        Transform targetWaypoint = waypoints[currentWaypointIndex];
        Vector3 direction = targetWaypoint.position - transform.position;

        if (isOnElevator)
        {
            // 엘리베이터와 함께 이동
            elevator.transform.Translate(direction.normalized * speed * Time.deltaTime, Space.World);
            transform.position = elevator.transform.position; // AGV를 엘리베이터 위치로 설정
        }
        else
        {
            // AGV만 이동
            transform.Translate(direction.normalized * speed * Time.deltaTime, Space.World);
        }

        if (Vector3.Distance(transform.position, targetWaypoint.position) < 0.1f)
        {
            if (currentWaypointIndex == 5 && !hasPickedUpPerson)  // 대기 지점
            {
                Debug.Log("pick up");
                hasPickedUpPerson = true;  // 사람이 따라가게 한 후에는 더 이상 이 코드가 실행되지 않게 함
                StartCoroutine(WaitAndMoveToNextWaypoint(5.0f));
            }
            else if (currentWaypointIndex == 2)
            {
                // Z축 기준으로 180도 회전
                transform.Rotate(0, 0, 180);
                // 엘리베이터와 같이 움직이기 시작
                isOnElevator = true;
                elevator.transform.position = transform.position; // AGV 위치로 엘리베이터 이동
                currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length;
            }
            else if (currentWaypointIndex == 3 || currentWaypointIndex == 8 || currentWaypointIndex == 11)
            {
                // 엘리베이터에서 내림
                isOnElevator = false;
                currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length;
            }
            else if (currentWaypointIndex == 9)  // 대기 지점
            {
                Debug.Log("drop off");
                StartCoroutine(WaitAndRotate(5.0f));
            }
            else if (currentWaypointIndex == 7)
            {   
                // 엘리베이터와 같이 움직이기 시작
                isOnElevator = true;
                elevator.transform.position = transform.position; // AGV 위치로 엘리베이터 이동
                currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length;
            }
            else if (currentWaypointIndex == 10)
            {
                transform.Rotate(0, 0, 180);
                // 엘리베이터와 같이 움직이기 시작
                isOnElevator = true;
                elevator.transform.position = transform.position; // AGV 위치로 엘리베이터 이동
                currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length;
            }
            else if (currentWaypointIndex == 13)  // 대기 지점
            {
                transform.Rotate(0, 0, 180);
                Debug.Log("arrived");
                StartCoroutine(WaitAndMoveToNextWaypoint(15.0f));
            }
            else
            {
                currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length;
            }
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        Debug.Log("Collision detected with: " + collision.gameObject.name);
        if (currentWaypointIndex == 5 && collision.gameObject.CompareTag("Person"))
        {
            FollowWaypoints followScript = collision.gameObject.AddComponent<FollowWaypoints>();
            followScript.waypoints = personWaypoints;
        }
    }

    void OnTriggerEnter(Collider other)
    {
        Debug.Log("Trigger detected with: " + other.gameObject.name);
        if (currentWaypointIndex == 5 && other.CompareTag("Person"))
        {
            FollowWaypoints followScript = other.gameObject.AddComponent<FollowWaypoints>();
            followScript.waypoints = personWaypoints;
        }
    }

    IEnumerator WaitAndMoveToNextWaypoint(float waitTime)
    {
        isMoving = false;
        yield return new WaitForSeconds(waitTime);
        currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length;
        isMoving = true;
    }

    private IEnumerator WaitAndRotate(float waitTime)
    {
        isMoving = false;
        yield return new WaitForSeconds(waitTime);  // 5초 대기
        transform.Rotate(0, 0, 180);  // 회전
        currentWaypointIndex = (currentWaypointIndex + 1) % waypoints.Length;
        isMoving = true;
    }
}
