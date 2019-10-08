# <img src="https://www.libreservo.eu/sites/libreservo.eu/files/imagenes/LibreServo_logo_xs.png">
An Open source controller to convert any servo motor to the best smart servo.

LibreServo ha nacido como necesidad propia de actualizar unos servomotores muy caros que tenía de hace años totalmente infrautilizados por tener una electrónica típica de los servomotores, básica y "tonta".
Con LibreServo, se pretenderá dotar de características típicas de "smart-servos" de más de 100 euros a cualquier servo de tamaños estándar.

    El servomotor podrá girar 360 grados.
    Se podrán encadenar los servomotores. No hará falta conectar todos los servomotores a la placa controladora.
    La resolución del servomotor será de 12bits como poco (4096 pasos).
    El rango de voltaje irá de tan sólo 4,5V hasta los 18V.
    La comunicación del servomotor podrá seguir siendo PWM para mantener la compatibilidad, pero para obtener todas las características, sería serie de hasta 9Mbps full duplex (configurable a diferentes velocidades) intentando en todo momento que no se necesite ningún tipo de hardware externo, siendo posible controlarlos directamente desde el pc (con un adaptador USB-Serie), con una arduino a 5V o con cualquier controlador a 3,3V.
    El propio servomotor será el encargado de generar las curvas de movimiento (por ejemplo senoidales o las potentes curvas hermíticas [dado punto inicial y final, así como las pendientes en dichos puntos, calcula la curva]).
    Capacidad de encadenar comandos de movimiento que el servo guardará e irá siguiendo de manera encadenada.
    Capacidad de leer la tensión, corriente consumida (instantánea, total y la media), posición y temperatura del servomotor.
    Capacidad de generar un log de posición, voltaje, corriente y/o temperatura para ser analizado a posteriori.
    Capacidad de poder alterar las constantes de control del servomotor.
    Capacidad de mandar el mismo comando a varios servomotores a la vez, a todos ellos o a rangos de ellos sin tener que enviar varias veces el comando, sólo una.
    Control del par motor mediante la lectura de la corriente.
    LED rgb (estamos en 2018, si algo no es rgb ya no vale para nada ;) ).

Más información en <a href="https://www.libreservo.com/">libreservo.com</a>
