#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "GUIPidControlWidget.hh"
#include "algorithm"
#include <boost/algorithm/string.hpp>

using namespace gazebo;
using namespace std;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GUIPidControlWidget)

/////////////////////////////////////////////////
GUIPidControlWidget::GUIPidControlWidget()
  : GUIPlugin()
{
  this->counter = 0;
   P = I = D = 0;

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//   Set the frame background and foreground colors
//  this->setStyleSheet(
//      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QHBoxLayout *modelLayout = new QHBoxLayout();
  QHBoxLayout *jointLayout = new QHBoxLayout();

  QLabel *modelLabel = new QLabel(tr("Model: "));
  QLabel *jointLabel = new QLabel(tr("Joint: "));

  // Create a push button, and connect it to the OnButton function
  QPushButton *button = new QPushButton(tr("Update PID"));
  connect(button, SIGNAL(clicked()), this, SLOT(OnButton()));

  this->comboModelBox = new QComboBox();
  this->comboJointBox = new QComboBox();
  this->comboModelBox->setMinimumSize(300, 25);
  this->comboJointBox->setMinimumSize(300, 25);

  modelLayout->addSpacing(10);
  modelLayout->addWidget(modelLabel);
  modelLayout->addWidget(this->comboModelBox);
  modelLayout->addSpacing(10);
  modelLayout->addStretch(4);

  jointLayout->addSpacing(10);
  jointLayout->addWidget(jointLabel);
  jointLayout->addSpacing(3);
  jointLayout->addWidget(this->comboJointBox);
  jointLayout->addSpacing(10);
  jointLayout->addStretch(4);

  //create P I D labels and edits
  QHBoxLayout *PidLayout = new QHBoxLayout;
  QLabel *PLabel = new QLabel(tr("P: "));
  QLabel *ILabel = new QLabel(tr("I: "));
  QLabel *DLabel = new QLabel(tr("D: "));

  this->pEdit = new QLineEdit;
  this->iEdit = new QLineEdit;
  this->dEdit = new QLineEdit;


  pEdit->setFixedWidth(80);
 // pEdit->setEchoMode(QLineEdit::Normal);

  iEdit->setFixedWidth(80);
  //iEdit->setEchoMode(QLineEdit::Normal);

  dEdit->setFixedWidth(80);
  //dEdit->setEchoMode(QLineEdit::Normal);

  PidLayout->addSpacing(10);
  PidLayout->addWidget(PLabel);
  PidLayout->addSpacing(5);
  PidLayout->addWidget(pEdit);
  PidLayout->addSpacing(4);
  PidLayout->addStretch(1);
  PidLayout->addWidget(ILabel);
  PidLayout->addWidget(iEdit);
  PidLayout->addSpacing(4);
  PidLayout->addStretch(1);
  PidLayout->addWidget(DLabel);
  PidLayout->addWidget(dEdit);
  PidLayout->addStretch(4);


  connect(this->comboModelBox, SIGNAL(currentTextChanged(const QString &)), this, SLOT(ModelItemChanged(const QString &)));
  connect(this->comboJointBox, SIGNAL(currentTextChanged(const QString &)), this, SLOT(JointItemChanged(const QString &)));
  //connect(this->pEdit, SIGNAL(editingFinished()), this, SLOT(ControlValueEntered()));

  // Add topicLayout to the frame
  mainLayout->addLayout(modelLayout);
  mainLayout->addLayout(jointLayout);
  mainLayout->addLayout(PidLayout);
  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);
  mainLayout->addWidget(button);

  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(8, 8, 8, 10);

  // Position and resize this widget
  this->move(10, 10);
  //this->resize(400, 100);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->requestMsg = msgs::CreateRequest("scene_info");

  this->requestPub = this->node->Advertise<msgs::Request>("~/request");
  this->requestPub->Publish(*this->requestMsg);

  this->responseSub = this->node->Subscribe("~/response", &GUIPidControlWidget::OnResponse, this);

  //TODO might be better to override showPopup and get active models at that time.
  //listen to advent indicating a new model has been inserted
  this->connections.push_back(gui::Events::ConnectModelUpdate(
              std::bind(&GUIPidControlWidget::OnModelUpdate, this, std::placeholders::_1)));

//  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
}

/////////////////////////////////////////////////
GUIPidControlWidget::~GUIPidControlWidget()
{
}

void GUIPidControlWidget::OnModelUpdate(const msgs::Model & msg)
{
    std::lock_guard<std::mutex> lock(this->mutex);
    cout << "model update " << endl;
    this->updateCombos(msg);
}
//called when model added
void GUIPidControlWidget::updateCombos(const msgs::Model & _msg)
{
    // Add new model to the list and it will be processed on Update
    //need to sleep a bit to let everything initialise
    bool result, Pid;
    string model_name = _msg.name();
    boost::replace_all(model_name, "::", "/");
    string topic = "/" + model_name + "/joint_cmd";
    string service = "/" + model_name + "/joint_cmd_req";

    ignition::msgs::StringMsg req;
    ignition::msgs::JointCmd rep;

    //eliminate joints that don't have active PID controllers. This is a bit hacky as all joints get initialised with w PID controller
    //can't determine which ones are active from pid proto msg. So just remove those with th defualt 1, 0.1, 0.01 -> look into lim value
    auto jointHasController = [&](string name) {
        vector<double> vals;
        vals.resize(3, 0);
        req.set_data(name);
        if (this->igNode.Request(service, req, 10, rep, result)) {
            //check if valid PID controller
            if (rep.has_position() && 
                    !(rep.position().p_gain_optional().data() == 1 &&
                      rep.position().i_gain_optional().data() == 0.1 &&
                      rep.position().d_gain_optional().data() == 0.01))
            {
                this->jointCtrType[name] = 1;
                vals[0]=(double)rep.position().p_gain_optional().data();
                vals[1]=(double)rep.position().i_gain_optional().data();
                vals[2]=(double)rep.position().d_gain_optional().data();
                this->PIDvalues[name] = vals;
                return true;
            }
            if (rep.has_velocity() && 
                    !(rep.velocity().p_gain_optional().data() == 1 &&
                      rep.velocity().i_gain_optional().data() == 0.1 &&
                      rep.velocity().d_gain_optional().data() == 0.01))
            {
                this->jointCtrType[name] = 2;
                vals[0]=(double)rep.velocity().p_gain_optional().data();
                vals[1]=(double)rep.velocity().i_gain_optional().data();
                vals[2]=(double)rep.velocity().d_gain_optional().data();
                this->PIDvalues[name] = vals;
                return true;
            }
        }
        return false;
    };

    //search through joints in model and add those with active PID controllers
    int idx = this->comboModelBox->findText(QString::fromStdString(_msg.name()));
    if (idx == -1) {
        this->comboModelBox->addItem(QString::fromStdString(_msg.name()));
        //check joints and add
        if (joints.find(_msg.name()) == joints.end()) {
            vector<QString> j;
            for (int i = 0; i < _msg.joint_size(); i++) {
                const msgs::Joint & joint = _msg.joint(i);
                if (!jointHasController(joint.name())) { continue; }
                j.push_back(QString::fromStdString(joint.name()));
            }
            if (!j.empty()) { joints[_msg.name()] = j; }
        }
    }
    else if (_msg.deleted()) {
        this->comboModelBox->removeItem(idx);
        joints.erase(_msg.name());
        return;
    }
    this->JointCtrPub = this->igNode.Advertise<ignition::msgs::JointCmd>(topic);
}

void GUIPidControlWidget::ModelItemChanged(const QString & name)
{
    std::lock_guard<std::mutex> lock(this->mutex);
    this->comboJointBox->clear();
    auto it = joints.find(name.toStdString());
    if (it == joints.end()) { return; }
    for (auto j : it->second) {
        this->comboJointBox->addItem(j);
    }
}

void GUIPidControlWidget::JointItemChanged(const QString & name)
{
    //TODO why was calling mutex lock causing crashing....
    //std::lock_guard<std::mutex> lock(this->mutex);
    if (PIDvalues.find(name.toStdString()) != PIDvalues.end()) {
        vector<double> v = PIDvalues[name.toStdString()];
        if (v.size() != 3) { return; }
        pEdit->setText(QString::number(v[0]));
        iEdit->setText(QString::number(v[1]));
        dEdit->setText(QString::number(v[2]));
    }
}

/////////////////////////////////////////////////
void GUIPidControlWidget::OnButton()
{
    //find topic name using model/joint
    std::lock_guard<std::mutex> lock(this->mutex);
    cout << "button clicked " << endl;
    //set P I D values
    this->P = this->pEdit->text().toDouble();
    this->I = this->iEdit->text().toDouble();
    this->D = this->dEdit->text().toDouble();

    //set up msg
    string joint_name  = this->comboJointBox->currentText().toStdString();
    ignition::msgs::JointCmd msg;
    msg.set_name(joint_name);
    if (this->jointCtrType[joint_name] == 1) {
        msg.mutable_position()->mutable_p_gain_optional()->set_data(this->P);
        msg.mutable_position()->mutable_i_gain_optional()->set_data(this->I);
        msg.mutable_position()->mutable_d_gain_optional()->set_data(this->D);
    }
    else if (this->jointCtrType[joint_name] == 2) {
        msg.mutable_velocity()->mutable_p_gain_optional()->set_data(this->P);
        msg.mutable_velocity()->mutable_i_gain_optional()->set_data(this->I);
        msg.mutable_velocity()->mutable_d_gain_optional()->set_data(this->D);
    }

    //Publish joint_cmd to update PID values
    JointCtrPub.Publish(msg);

    //update map with new PID values
    auto it = PIDvalues.find(joint_name);
    if (it != PIDvalues.end()) {
        it->second[0] = this->P;
        it->second[1] = this->I;
        it->second[2] = this->D;
    }
}

//called when scene initially loaded
void GUIPidControlWidget::OnResponse(ConstResponsePtr & _msg)
{
    // Check it is the response to our request
    if (!this->requestMsg || _msg->id() != this->requestMsg->id()) { return; }

    // Verify message type
    msgs::Scene sceneMsg;
    if (_msg->has_type() && _msg->type() != sceneMsg.GetTypeName())
    return;

    // Get scene data
    sceneMsg.ParseFromString(_msg->serialized_data());

    // Fill vector with models in the scene
    std::lock_guard<std::mutex> lock(this->mutex);
    for (int i = 0; i < sceneMsg.model_size(); ++i) {
        this->updateCombos(sceneMsg.model(i));
    }
}

