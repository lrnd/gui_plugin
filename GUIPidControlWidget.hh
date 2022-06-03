#ifndef _GUI_PID_CONTROL_WIDGET_HH_
#define _GUI_PID_CONTROL_WIDGET_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
// moc parsing error of tbb headers
#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#endif
#include <gazebo/gui/gui.hh>

namespace gazebo
{
    class GAZEBO_VISIBLE GUIPidControlWidget : public GUIPlugin
    {
      Q_OBJECT

      // Constructor
      public: GUIPidControlWidget();

      // Destructor
      public: virtual ~GUIPidControlWidget();

      // Callback trigged when the button is pressed.
      protected slots: void OnButton();

      // Callback for when model name changed
      protected slots: void ModelItemChanged(const QString & name);
      protected slots: void JointItemChanged(const QString & name);

      // Callback for response msg
      private: void OnResponse(ConstResponsePtr & _msg);

      // Callback for pre-render event
//      private: void OnUpdate();

      // Callback for model update event
      private: void OnModelUpdate(const msgs::Model & msg);

      // Function to Update Combo boxes
      private: void updateCombos(const msgs::Model & _msg);

      // Counter used to create unique model names
      private: unsigned int counter;

      // Node used to establish communication with gzserver.
      private: transport::NodePtr node;

      //Node used for Ignition msg type
      private: ignition::transport::Node igNode;
      private: ignition::transport::Node::Publisher JointCtrPub;


      // Publisher of factory messages.
      private: transport::PublisherPtr requestPub;
      
      // Subscribe to request msg
      private: transport::SubscriberPtr responseSub;
      
      // Message to be used to make requests
      private: msgs::Request *requestMsg;
    
      // 
      //hold all joint names from models TODO this needs to be map<string, vector<QString>> so i can get the data later
      private: std::map<std::string, std::vector<QString>> joints;
      private: std::map<std::string, int> jointCtrType; // 0 = none, 1 = position, 2 = velocity
      private: std::map<std::string, std::vector<double>> PIDvalues;
      
      // QComboBox to hold model names
      private: QComboBox * comboModelBox;
      private: QComboBox * comboJointBox;
      private: QLineEdit * pEdit;
      private: QLineEdit * iEdit;
      private: QLineEdit * dEdit;

      // All event connections
      private: std::vector<event::ConnectionPtr> connections;
      private: event::ConnectionPtr updates;
    
      // Mutex
      private: std::mutex mutex;

      //varibles
      private:
        double P, I, D;
    };
}
#endif
