#include <ros/ros.h> // nous donne l'accès aux fonctionnalités et objets natifs de ROS
#include <cmath> // nous permettra de récupérer la valeur absolue de l'effort
#include <nxt_msgs/JointCommand.h> // récupère le message JointCommand du package nxt_msgs
#include <nxt_adventurer/Order.h> // récupère le message que nous avons créé précédement

class Adventurer
{
public:
    Adventurer()  { init(); }
    ~Adventurer() { ; }

private:
    // permet d'intéragir avec ros (récupération des parametre, déclaration des subscriber, des publisher, etc.
    ros::NodeHandle   m_NodeHandle;
    
    // représente le publisher permettant de publier des ordres aux moteurs
    ros::Publisher    m_WheelsPublisher;
    // représente le subscriber recevant des ordresde notre autre application
    ros::Subscriber   m_OrderSubscriber;
    
    // nom du moteur droit
    std::string       m_sRightWheel;
    //nom du moteur gauche
    std::string       m_sLeftWheel;

    // initialise les variables du noeud
    void init();
    // callback appellé à la reception d'un ordre
    void order_cb(const nxt_adventurer::Order::ConstPtr& msg);
    
    // fait avancer ou reculer le robot
    void move(float fEffort = 0.0, bool bAhead = true);
    // fait tourner le robots à droite ou à gauche
    void turn(float fEffort = 0.0, bool bRight = true);    
};

void Adventurer::init()
{
    // strings temporaires
    std::string sSubscribeName,
                sRightWheel,
                sLeftWheel;

    // essaye de récupérer le nom du topic sur lequel on recoie les ordres
    // si aucun parametre n'est pas trouvé, on utilise une valeur par defaut
    if (!m_NodeHandle.getParam("sub_name", sSubscribeName))
        sSubscribeName = "adventurer_order"; // valeur par defaut
    
    // création du subscriber pour recevoir les ordres
    // on passe en parametre la callback (order_cb) et l'objet qui l'appellera (this)
    m_OrderSubscriber = m_NodeHandle.subscribe(sSubscribeName, 1, &Adventurer::order_cb, this);
    // création du publisher pour envoyer des ordres au robot
    m_WheelsPublisher = m_NodeHandle.advertise<nxt_msgs::JointCommand>("joint_command", 1);

    // cf. ligne 42
    if (!m_NodeHandle.getParam("right_wheel", sRightWheel))
        sRightWheel = "motor_r";
    if (!m_NodeHandle.getParam("left_wheel", sLeftWheel))
        sLeftWheel  = "motor_l";

    m_sRightWheel = sRightWheel;
    m_sLeftWheel  = sLeftWheel;
}

void Adventurer::order_cb(const nxt_adventurer::Order::ConstPtr& msg)
{
    if (msg->order == 0)
        move(msg->effort, msg->direction);
    else
        turn(msg->effort, msg->direction);
}

void Adventurer::move(float fEffort, bool bAhead)
{
    // création de deux messages <JointCommand> (1 pour le moteur droit et 1 pour le moteur gauche)
    nxt_msgs::JointCommand rightCommand;
    nxt_msgs::JointCommand leftCommand;

    // assignation des noms 
    rightCommand.name = m_sRightWheel;
    leftCommand.name  = m_sLeftWheel;

    // pour un déplacement linéaire on assigne le même effort aux deux moteurs
    if (bAhead)
        rightCommand.effort = leftCommand.effort = std::abs(fEffort);
    else
        rightCommand.effort = leftCommand.effort = -std::abs(fEffort);

    // on publie
    m_WheelsPublisher.publish(rightCommand);
    m_WheelsPublisher.publish(leftCommand);
}

void Adventurer::turn(float fEffort, bool bRight)
{
    // cf. ligne 73
    nxt_msgs::JointCommand rightCommand;
    nxt_msgs::JointCommand leftCommand;
    
    rightCommand.name = m_sRightWheel;
    leftCommand.name  = m_sLeftWheel;

    // pour tourner on applique un effort inverse sur chaque moteur
    if (bRight)
    {
        rightCommand.effort = -fEffort;
        leftCommand.effort  = fEffort;
    }
    else
    {
        rightCommand.effort = fEffort;
        leftCommand.effort  = -fEffort;
    }

    m_WheelsPublisher.publish(rightCommand);
    m_WheelsPublisher.publish(leftCommand);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nxt_adventurer");
    Adventurer a;
    ros::spin();
    return 0;
}
