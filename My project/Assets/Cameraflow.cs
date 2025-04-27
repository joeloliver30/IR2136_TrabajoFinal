using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Cameraflow : MonoBehaviour
{
    // Start is called before the first frame update
    public Transform dron;    // Referencia al objeto dron
    public Vector3 offset;    // Offset de la cámara respecto al dron

    void Start()
    {
        // Definir el offset de la cámara (ajusta esto para que quede bien)
        offset = new Vector3(0, 10, -15);  // La cámara está 2 unidades arriba y 5 unidades detrás del dron
    }

    void LateUpdate()
    {
        
        // Actualizar la posición de la cámara para que siga al dron
        transform.position = dron.position + offset;
        // Para que la cámara mantenga la orientación del dron, ajustamos la rotación
        transform.rotation = Quaternion.Euler(0, dron.eulerAngles.y, 0);

        // Hacer que la cámara mire siempre hacia el dron
        transform.LookAt(dron);
    }
}
