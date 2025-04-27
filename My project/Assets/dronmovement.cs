using System.Collections;
using UnityEngine;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Std;

public class AterrizaDron : MonoBehaviour
{
    public float velocidad = 20f;        // Velocidad de movimiento del dron
    public float rotacionVel = 100f;     // Velocidad de rotación del dron
    public float subidaVel = 10f;        // Velocidad de subida y bajada
    public float alturaAterrizaje = 1f;  // Altura a la que aterrizará el dron
    public float distanciaDeAterrizaje = 1f; // Distancia mínima al suelo para detenerse
    private float altura_ant = 0f;
    
    private Rigidbody rb;                // Referencia al Rigidbody
    private bool aterrizando = false;    // Indica si el dron está aterrizando
    private bool haAterrizado = false;   // **Nueva variable para evitar múltiples aterrizajes**
    
    private float nivelBateria = 100f;   // Nivel actual de batería
    public RosConnector rosConnector;  // Referencia pública a RosConnector
    
    private string topicName = "/battery_status"; // Nombre del tópico de la batería

    void Start()
    {
        // Obtener el Rigidbody del dron
        rb = GetComponent<Rigidbody>();

        // Verificar si el Rigidbody está asignado correctamente
        if (rb == null)
        {
            Debug.LogError("El Rigidbody no está asignado al dron. Asegúrate de que el objeto tiene un Rigidbody.");
        }

        // Intentar obtener el componente RosConnector
        rosConnector = GetComponent<RosConnector>();
        
        if (rosConnector != null)
        {
            rosConnector.RosSocket.Subscribe<Float32>(topicName, BatteryStatusCallback);
        }
        else
        {
            Debug.LogError("No se encontró el componente RosConnector.");
        }
    }

    void Update()
    {
        if (aterrizando || haAterrizado)  // **Si ya aterrizó, no hacer nada**
        {
            return;
        }

        MoveDron();

        if (nivelBateria < 80f && !aterrizando && !haAterrizado)  // **Evitar múltiples aterrizajes**
        {
            Debug.Log("Nivel de batería bajo. Aterrizando el dron...");
            aterrizando = true;
            StartCoroutine(Aterrizar());
        }
    }

    void MoveDron()
    {
        float moveVertical = Input.GetAxis("Vertical");
        Vector3 movimiento = transform.forward * moveVertical * velocidad * Time.deltaTime;

        float moveHorizontal = Input.GetAxis("Horizontal");
        float rotar = moveHorizontal * rotacionVel * Time.deltaTime;

        float subirBajar = 0f;
        if (Input.GetKey(KeyCode.Q)) subirBajar = 1f;
        if (Input.GetKey(KeyCode.E)) subirBajar = -1f;
        Vector3 subida = transform.up * subirBajar * subidaVel * Time.deltaTime;

        rb.MovePosition(rb.position + movimiento + subida);
        rb.MoveRotation(rb.rotation * Quaternion.Euler(0, rotar, 0));
    }

    void BatteryStatusCallback(Float32 batteryLevel)
    {
        nivelBateria = batteryLevel.data;
        Debug.Log("Nivel de batería: " + nivelBateria);
    }

    IEnumerator Aterrizar()
    {
        float velocidadDescensoMax = 0.1f;
        float velocidadDescensoMin = 0.0001f;

        while ((transform.position.y > alturaAterrizaje) && aterrizando)
        {   
            float distanciaAlSuelo = transform.position.y - alturaAterrizaje;
            float velocidadDescenso = Mathf.Lerp(velocidadDescensoMin, velocidadDescensoMax, distanciaAlSuelo / 10f);
            velocidadDescenso = Mathf.Max(velocidadDescenso, velocidadDescensoMin);

            rb.MovePosition(rb.position - new Vector3(0, velocidadDescenso, 0)); 
            
            if (Mathf.Abs(rb.position.y - altura_ant) < 0.01f)
            {
                aterrizando = false;
            }
            altura_ant = rb.position.y;
            Debug.Log("Anterior altura: " + altura_ant);

            yield return null;
        }

        // **Asegurar que el dron se detiene completamente**
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        
        
        haAterrizado = true;  // **Marcar que el dron ya aterrizó**
        Debug.Log("El dron ha aterrizado completamente.");
    }
}
