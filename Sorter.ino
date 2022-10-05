#include <Robot.h>
#include <DFRobot_ICG20660L.h>
/* moteur */
#define ENABLEXY 8 // enable les moteurs
#define GOTIMES 9  // nombre de billes à trier

/* delays */
#define DELAYSTEPS 500 // delay entre chaque step (microseconde)
#define COLORREAD 20   // delay entre chaque lecture de couleur (millis)

struct Moteur_s
{
    int stepPin;         // pin des steps
    int dirPin;          // pin de direction
    int current_loc;     // location actuelle
    int switch_assigned; // switch assignée
    bool is_initialized; // si le moteur est initialisé
    bool direction;      // direction du moteur
    // pin step, pin dir, switch assignée
    Moteur_s(int stepPin, int dirPin, int switch_assigned) : stepPin(stepPin), dirPin(dirPin), current_loc(0), switch_assigned(switch_assigned), is_initialized(false), direction(true){};
    // pin step, pin dir, switch assignée, direction
    Moteur_s(int stepPin, int dirPin, int switch_assigned, bool switch_dir) : stepPin(stepPin), dirPin(dirPin), current_loc(0), switch_assigned(switch_assigned), is_initialized(false), direction(switch_dir){};
    // utiliser dans le void setup, par example: moteur[0].setup();
    void setup()
    {
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        pinMode(switch_assigned, INPUT);
    }
    // on met le moteur a zero, par exemple: moteur[0].init(1, 1);
    void init(int steps, unsigned int delay_multiplier)
    {
        // si steps est 0 on émet un message d'erreur
        Serial.println("initializing");
        if (steps == 0)
        {
            Serial.println("Watch out, steps is 0 on init, motor will set home to current location");
            is_initialized = true; // on le set a init meme si on sait pas ou on est
            current_loc = 0;
            return;
        }
        // on check la limit switch et on bouge si elle est pas activée
        while (digitalRead(switch_assigned) == LOW)
        {

            bouge_and_off(steps, delay_multiplier); // on bouge moteur 1 par 1 step dans la direction de la limit switch
        }
        current_loc = 0;
        is_initialized = true; // on le set a init
    }
    // on bouge le moteur et update la location, delay multiplier va changer l'ampleur du delay
    void bouge_and_off(int steps, unsigned int delay_multiplier)
    {
        if (steps == 0) // sorti plus rapide en cas d'aucun mouvement
        {
            return;
        }
        digitalWrite(ENABLEXY, HIGH);                                          // on enable le moteur
        digitalWrite(dirPin, constrain(((direction) ? steps : -steps), 0, 1)); // set la direction du moteur
        for (int i = 0; i < abs(steps); i++)                                   // on répete par le nombre de steps
        {
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(DELAYSTEPS * delay_multiplier);
            digitalWrite(stepPin, LOW * delay_multiplier);
            delayMicroseconds(DELAYSTEPS);
        }
        current_loc += steps;        // on update la location
        digitalWrite(ENABLEXY, LOW); // on disable le moteur
    }
    // Va a step par rapport au zero, init au besoin (unsafe)
    void goto_step(int location, unsigned int delay_multiplier)
    {
        if (is_initialized == false) // on check si il est init
        {
            Serial.println("moteur is not init, initializing (may be unsafe) from goto_step");
            init(1, 1); // on init le moteur, fonctionne juste si la switch est du bon bord pour plus de contrôle utiliser init()
        }
        int movement = location - current_loc;     // trouve le nombre de steps vers le composant
        bouge_and_off(movement, delay_multiplier); // va au composant
    }
    // fait un certain nombre de steps vers les steps assignés ne fait rien si pas init
    void travel(int destination, unsigned int steps, unsigned int delay_multiplier)
    {
        if (is_initialized == false) // on check si il est init
        {
            Serial.println("moteur is not init, not traveling");
            return;
        }
        // trouve la direction vers la destination
        bool direction = (destination > current_loc) ? true : false;
        // bouge_and_off le nombre de steps vers la destination
        bouge_and_off((direction) ? steps : -steps, delay_multiplier);
    }
    // bouge vers la destination pendant un certain temps si pas init ne fait rien
    void travel_ms(int destination, unsigned int delay_ms, unsigned int delay_multiplier)
    {
        if (is_initialized == false) // on check si il est init
        {
            Serial.println("moteur is not init, not traveling");
            return;
        }
        unsigned long start_time = millis(); // on prend le temps actuel
        // on bouge tant que le temps est pas depassé
        while (millis() - start_time < delay_ms)
        {
            travel(destination, 1, delay_multiplier); // on bouge 1 step vers la destination
        }
    }
};
// exemple
// double_goto(moteur[0], moteur[1], 30, 40); moteur 0 va a 30 et moteur 1 va a 40
// fait un goto sur deux moteurs en meme temps et init au besoin (unsafe)
void double_goto(Moteur_s &motor1, Moteur_s &motor2, int location1, int location2, unsigned int delay_multiplier)
{
    // on trouve le nombre de steps pour chaque moteur
    if (motor1.is_initialized == false) // on check si il est init
    {
        Serial.println("moteur1 is not init, initializing (may be unsafe) from double_goto");
        motor1.init(1, 1); // on init le moteur, fonctionne juste si la switch est du bon bord pour plus de contrôle utiliser init()
    }
    if (motor2.is_initialized == false) // on check si il est init
    {
        Serial.println("moteur2 is not init, initializing (may be unsafe) from double_goto");
        motor2.init(1, 1); // on init le moteur, fonctionne juste si la switch est du bon bord pour plus de contrôle utiliser init()
    }
    int movement1 = location1 - motor1.current_loc;                                            // steps requis pour moteur 1
    int movement2 = location2 - motor2.current_loc;                                            // steps requis pour moteur 2
    int repeate = abs(movement1) > abs(movement2) ? abs(movement1) : abs(movement2);           // répétition requise
    digitalWrite(motor1.dirPin, constrain((motor1.direction) ? movement1 : -movement1, 0, 1)); // direction du moteur 1
    digitalWrite(motor2.dirPin, constrain((motor2.direction) ? movement2 : -movement2, 0, 1)); // direction du moteur 2
    digitalWrite(ENABLEXY, HIGH);                                                              // enable les moteurs
    // for loop de la variable repeate
    for (int i = 0; i < abs(repeate); i++)
    {
        if (motor1.current_loc != location1) // si le moteur 1 n'est pas a la location
        {
            digitalWrite(motor1.stepPin, HIGH); // on enable le moteur 1
        }
        if (motor2.current_loc != location2) // si le moteur 2 n'est pas a la location
        {
            digitalWrite(motor2.stepPin, HIGH); // on enable le moteur 2
        }
        delayMicroseconds(DELAYSTEPS * delay_multiplier); // on attend un peu (pour tout les moteurs en meme temps)
        if (motor1.current_loc != location1)              // si le moteur 1 n'est pas a la location
        {
            digitalWrite(motor1.stepPin, LOW); // on disable le moteur 1
            motor1.current_loc += constrain(movement1, 0, 1) * 2 - 1;
        }
        if (motor2.current_loc != location2) // si le moteur 2 n'est pas a la location
        {
            digitalWrite(motor2.stepPin, LOW); // on disable le moteur 2
            motor2.current_loc += constrain(movement2, 0, 1) * 2 - 1;
        }
    }
    digitalWrite(ENABLEXY, LOW);
}
// goto pour m1 et m2 va se diriger vers la location ou on veut
void goto_and_trav(Moteur_s &set_motor, Moteur_s &trav_motor, int location1, int location2, unsigned int delay_multiplier)
{
    // on trouve le nombre de steps pour chaque moteur
    if (set_motor.is_initialized == false) // on check si il est init
    {
        Serial.println("set_motor is not init, initializing (may be unsafe)");
        set_motor.init(1, 1); // on init le moteur, fonctionne juste si la switch est du bon bord pour plus de contrôle utiliser init()
    }
    if (trav_motor.is_initialized == false) // on check si il est init
    {
        Serial.println("trav_motor is not init, initializing (may be unsafe)");
        trav_motor.init(1, 1); // on init le moteur, fonctionne juste si la switch est du bon bord pour plus de contrôle utiliser init()
    }
    int movement1 = location1 - set_motor.current_loc;                                                 // steps requis pour moteur 1
    int movement2 = location2 - trav_motor.current_loc;                                                // steps requis pour moteur 2
    digitalWrite(set_motor.dirPin, constrain((set_motor.direction) ? movement1 : -movement1, 0, 1));   // direction du moteur 1
    digitalWrite(trav_motor.dirPin, constrain((trav_motor.direction) ? movement2 : -movement2, 0, 1)); // direction du moteur 2
    digitalWrite(ENABLEXY, HIGH);                                                                      // enable les moteurs
    // for loop de la abs du mouvement 1
    for (int i = 0; i < abs(movement1); i++)
    {
        if (set_motor.current_loc != location1) // si le moteur 1 n'est pas a la location
        {
            digitalWrite(set_motor.stepPin, HIGH); // on enable le moteur 1
        }
        if (trav_motor.current_loc != location2) // si le moteur 2 n'est pas a la location
        {
            digitalWrite(trav_motor.stepPin, HIGH); // on enable le moteur 2
        }
        delayMicroseconds(DELAYSTEPS * delay_multiplier); // on attend un peu (pour tout les moteurs en meme temps)
        if (set_motor.current_loc != location1)           // si le moteur 1 n'est pas a la location
        {
            digitalWrite(set_motor.stepPin, LOW);                        // on disable le moteur 1
            set_motor.current_loc += constrain(movement1, 0, 1) * 2 - 1; // on update la location du moteur 1
        }
        if (trav_motor.current_loc != location2) // si le moteur 2 n'est pas a la location
        {
            digitalWrite(trav_motor.stepPin, LOW);                        // on disable le moteur 2
            trav_motor.current_loc += constrain(movement2, 0, 1) * 2 - 1; // on update la location du moteur 2
        }
    }
    digitalWrite(ENABLEXY, LOW);
}
// permet de faire un goto sur toute les moteurs en meme temps init les moteurs au besoin (unsafe)
void multi_goto(Moteur_s motors[], int locations[], int motors_len, unsigned int delay_multiplier)
{
    // set le mouvement a faire pour chaque moteur et les initialise au besoin
    int movements[motors_len];
    for (int i = 0; i < motors_len; i++)
    {
        movements[i] = locations[i] - motors[i].current_loc;
        if (motors[i].is_initialized == false)
        {
            Serial.print("moteur");
            Serial.print(i);
            Serial.println(" is not init, initializing (may be unsafe) from multi_goto");
            motors[i].init(1, 1);
        }
    }
    // trouve le nombre de répétition a faire
    int max_movement = 0;
    for (int i = 0; i < motors_len; i++)
    {
        if (abs(movements[i]) > max_movement)
        {
            max_movement = abs(movements[i]);
        }
    }
    // skip si le max movement est 0
    if (max_movement == 0)
    {
        return;
    }
    // set la direction de chaque moteur
    for (int i = 0; i < motors_len; i++)
    {
        digitalWrite(motors[i].dirPin, constrain((motors[i].direction) ? movements[i] : -movements[i], 0, 1));
    }
    // enable the motors
    digitalWrite(ENABLEXY, HIGH);
    // for loop of max_movement
    for (int i = 0; i < abs(max_movement); i++)
    {
        // on affecte juste les moteurs qui doivent bouger
        for (int j = 0; j < motors_len; j++)
        {
            if (motors[j].current_loc != locations[j])
            {
                digitalWrite(motors[j].stepPin, HIGH);
            }
        }
        delayMicroseconds(DELAYSTEPS * delay_multiplier);
        // on affecte juste les moteurs qui doivent bouger
        for (int j = 0; j < motors_len; j++)
        {
            if (motors[j].current_loc != locations[j])
            {
                digitalWrite(motors[j].stepPin, LOW);
                motors[j].current_loc += constrain(movements[j], 0, 1) * 2 - 1;
            }
        }
    }
}
// permet de faire un goto sur toute les moteurs en meme temps init au besoin et init les moteurs qui font rien si possible (unsafe)
void multi_goto_spare(Moteur_s motors[], int locations[], unsigned int motors_len, unsigned int init_trying_threshold, unsigned int delay_multiplier)
{
    // on set les moteur tout a non init dans notre contexte
    bool is_initialized[motors_len];
    for (int i = 0; i < motors_len; i++)
    {
        is_initialized[i] = false;
    }
    bool is_initializing[motors_len];
    for (int i = 0; i < motors_len; i++)
    {
        is_initializing[i] = false;
    }
    // set le mouvement a faire pour chaque moteur et les initialise au besoin
    int movements[motors_len];
    for (int i = 0; i < motors_len; i++)
    {
        movements[i] = locations[i] - motors[i].current_loc;
        if (motors[i].is_initialized == false)
        {
            Serial.print("moteur");
            Serial.print(i);
            Serial.println(" is not init, initializing (may be unsafe) from multi_goto_spare");
            motors[i].init(1, 1);     // ici on init le moteur qui ne l'est pas
            is_initialized[i] = true; // pas besoin de le reinit dans la meme fonction
        }
    }
    // trouve le nombre de répétition a faire
    int max_movement = 0;
    for (int i = 0; i < motors_len; i++)
    {
        if (abs(movements[i]) > max_movement)
        {
            max_movement = abs(movements[i]);
        }
    }
    // skip si le max movement est 0
    if (max_movement == 0)
    {
        return;
    }
    // set la direction de chaque moteur
    for (int i = 0; i < motors_len; i++)
    {
        digitalWrite(motors[i].dirPin, constrain((motors[i].direction) ? movements[i] : -movements[i], 0, 1));
    }
    // enable the motors
    digitalWrite(ENABLEXY, HIGH);
    // for loop of max_movement
    for (int i = 0; i < abs(max_movement); i++)
    {
        // on affecte juste les moteurs qui doivent bouger
        for (int j = 0; j < motors_len; j++)
        {
            if (motors[j].current_loc != locations[j])
            {
                digitalWrite(motors[j].stepPin, HIGH);
            }
            //
            else if (((abs(abs(motors[j].current_loc) - abs(max_movement) - i) < init_trying_threshold) || is_initializing[j]) && is_initialized[j] == false) // check si on peut essayer de init
            {
                // set la direction du moteur vers la limit switch
                is_initializing[j] = true;
                digitalWrite(motors[j].dirPin, HIGH); // possible source of error
                digitalWrite(motors[j].stepPin, HIGH);
            }
        }
        delayMicroseconds(DELAYSTEPS * delay_multiplier);
        // on affecte juste les moteurs qui doivent bouger
        for (int j = 0; j < motors_len; j++)
        {
            if (motors[j].current_loc != locations[j])
            {
                digitalWrite(motors[j].stepPin, LOW);
                motors[j].current_loc += constrain(movements[j], 0, 1) * 2 - 1;
            }
            else if (((abs(abs(motors[j].current_loc) - abs(max_movement) - i) < init_trying_threshold) || is_initializing[j]) && is_initialized[j] == false)
            {
                // check limit switch
                if (digitalRead(motors[j].switch_assigned) == HIGH)
                {
                    // set le moteur a init
                    is_initialized[j] = true;   // set le moteur a initialised
                    is_initializing[j] = false; // set le moteur a pas initialising
                    motors[j].current_loc = 0;  // set le current_loc a 0
                }
                else
                {
                    motors[j].current_loc--; // set loc a current_loc - 1
                }
                digitalWrite(motors[j].stepPin, LOW); // étein le moteur
            }
        }
    }
}
// bouge 1 moteur et initialise les autres si possible
void goto_spare(unsigned int motor_number, Moteur_s motors[], int location, unsigned int motors_len, unsigned int init_trying_threshold, unsigned int delay_multiplier)
{
    // on set les moteur tout a non init dans notre contexte
    bool is_initialized[motors_len];
    for (int i = 0; i < motors_len; i++)
    {
        is_initialized[i] = false;
    }
    bool is_initializing[motors_len];
    for (int i = 0; i < motors_len; i++)
    {
        is_initializing[i] = false;
    }
    // set le mouvement a faire pour chaque moteur et les initialise au besoin
    int movement = location - motors[motor_number].current_loc;
    for (int i = 0; i < motors_len; i++)
    {
        if (motors[i].is_initialized == false)
        {
            Serial.print("moteur");
            Serial.print(i);
            Serial.println(" is not init, initializing (may be unsafe) from multi_goto");
            motors[i].init(1, 1);     // ici on init le moteur qui ne l'est pas
            is_initialized[i] = true; // pas besoin de le reinit dans la meme fonction
        }
    }
    // skip si le max movement est 0
    if (movement == 0)
    {
        return;
    }
    // set la direction de chaque moteur
    for (int i = 0; i < motors_len; i++)
    {
        digitalWrite(motors[i].dirPin, constrain((motors[i].direction) ? movement : -movement, 0, 1));
    }
    // enable the motors
    digitalWrite(ENABLEXY, HIGH);
    // for loop of max_movement
    for (int i = 0; i < abs(movement); i++)
    {
        // on affecte juste les moteurs qui doivent bouger
        for (int j = 0; j < motors_len; j++)
        {
            if (j == motor_number)
            {
                digitalWrite(motors[motor_number].stepPin, HIGH);
            }
            else if (((abs(abs(motors[j].current_loc) - abs(movement) - i) < init_trying_threshold) || is_initializing[j]) && is_initialized[j] == false) // check si on peut essayer de init
            {
                // set la direction du moteur vers la limit switch
                is_initializing[j] = true;
                digitalWrite(motors[j].dirPin, HIGH);
                digitalWrite(motors[j].stepPin, HIGH);
            }
        }
        delayMicroseconds(DELAYSTEPS * delay_multiplier);
        // on affecte juste les moteurs qui doivent bouger
        for (int j = 0; j < motors_len; j++)
        {
            if (j == motor_number)
            {
                digitalWrite(motors[j].stepPin, LOW);
                motors[motor_number].current_loc += constrain(movement, 0, 1) * 2 - 1;
            }
            else if (((abs(abs(motors[j].current_loc) - abs(movement) - i) < init_trying_threshold) || is_initializing[j]) && is_initialized[j] == false)
            {
                // check limit switch
                if (digitalRead(motors[j].switch_assigned) == HIGH)
                {
                    is_initialized[j] = true;   // set le moteur a initialised
                    is_initializing[j] = false; // set le moteur a pas initialising
                    motors[j].current_loc = 0;  // set le current_loc a 0
                }
                else
                {
                    motors[j].current_loc--; // set loc a current_loc - 1
                }
                digitalWrite(motors[j].stepPin, LOW); // étein le moteur
            }
        }
    }
}
// bouge toute les moteurs en meme temps len est le nombre de moteurs dans motors
void multi_bouge(Moteur_s motors[], int steps[], int motors_len, unsigned int delay_multiplier)
{
    // on setup la direction des moteurs
    for (int i = 0; i < motors_len; i++)
    {
        digitalWrite(motors[i].dirPin, constrain((motors[i].direction) ? steps[i] : -steps[i], 0, 1));
    }
    // on trouve le nombre de steps max
    int max_steps = 0;
    for (int i = 0; i < motors_len; i++)
    {
        if (abs(steps[i]) > max_steps)
        {
            max_steps = abs(steps[i]);
        }
    }
    // skip si le max steps est 0
    if (max_steps == 0)
    {
        return;
    }
    // on enable les moteurs
    digitalWrite(ENABLEXY, HIGH);
    // on bouge les moteurs
    for (int i = 0; i < abs(max_steps); i++)
    {
        // on affecte juste les moteurs qui doivent bouger
        for (int j = 0; j < motors_len; j++)
        {
            if (steps[j] != 0)
            {
                digitalWrite(motors[j].stepPin, HIGH);
            }
        }
        delayMicroseconds(DELAYSTEPS * delay_multiplier);
        // on affecte juste les moteurs qui doivent bouger
        for (int j = 0; j < motors_len; j++)
        {
            if (steps[j] != 0)
            {
                digitalWrite(motors[j].stepPin, LOW);
                motors[j].current_loc += constrain(steps[j], 0, 1) * 2 - 1;
                steps[j] += constrain(steps[j], 0, 1) * 2 - 1;
            }
        }
    }
}
// init toute les moteurs qui ne sont pas init
void multi_init(Moteur_s motors[], int speed[], int motors_len, unsigned int delay_multiplier)
{
    for (int i = 0; i < motors_len; i++)
    {
        if (motors[i].is_initialized == false)
        {
            motors[i].init((motors[i].direction) ? speed[i] : -speed[i], delay_multiplier);
        }
    }
}
// init toute les moteurs meme si ils sont deja init
void multi_force_init(Moteur_s motors[], int speed[], int motors_len, unsigned int delay_multiplier)
{
    for (int i = 0; i < motors_len; i++)
    {
        motors[i].init((motors[i].direction) ? speed[i] : -speed[i], delay_multiplier);
    }
}
// multi setup des pins
void multi_setup(Moteur_s motors[], int motors_len)
{
    for (int i = 0; i < motors_len; i++)
    {
        motors[i].setup();
    }
}
// /* Declaration des composantes que l'on a */
Tcs230 sensor(52, 50, 48, 46, 2, 44, 15, 5);                                // sensor de couleur, pin fs0 fs0 cs0 cs1 out led treshold repeat
Moteur_s moteur[] = {Moteur_s(3, 6, 53, false), Moteur_s(4, 7, 51, false)}; // pour dealer avec multi_goto il faut un array de moteurs 0 = haut 1 = bas
int len = *(&moteur + 1) - moteur;                                          // trouve le montant de moteurs automatiquement pas besoin de changer
void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
    }
    robot.setup();
    /* Pour imprimer les erreurs*/
    Serial.begin(9600);
    /* Initialisation du sensor */
    sensor.sensor_setup(); // setup du sensor
    /* Frequency scaling  */
    sensor.frequency_setup(HIGH, LOW); // 20%
    /* moteurs */
    pinMode(ENABLEXY, OUTPUT);             // enable pin
    multi_setup(moteur, len);              // setup des pins de tout les moteurs
    int direction[2] = {1, 1};             // montant de steps vers la switch chaque fois (je suggere de mettre a 1 et de changer la direction des moteur au besoin)
    multi_init(moteur, direction, len, 2); // setup plusieurs moteurs en meme temps, hautement recommande
    Serial.println("done init");
}

/*compte avant qu'il se réinitialise*/
unsigned int count = 0;
void loop()
{
    if (count % GOTIMES == 0 && count != 0)
    {
        while (1)
        {
        } // arrête de trier les billes
    }

    goto_and_trav(moteur[0], moteur[1], BILLE_LOC, MIDDLE, 3);        // on va à la bille et en meme temps on travel vers millieu
    moteur[1].travel_ms(MIDDLE, 100, 1);                              // on attend sinon la bille a pas le temps de tomber en meme temps on travel vers millieu
    goto_and_trav(moteur[0], moteur[1], SENS_LOC, MIDDLE, 3);         // on apporte la bille au lecteur de couleur et en meme temps on travel vers millieu
    int locations[2] = {TROU_LOC, sensor.rgb_to_step(sensor.read())}; // on fait un array des locations ou ont veut aller
    multi_goto(moteur, locations, len, 3);                            // bouge toute les moteurs en meme temps
    moteur[0].travel_ms(BILLE_LOC, 700, 3);                           // delay pour laisser la bille glisser en meme temps on travel vers la bille
    count++;
}
