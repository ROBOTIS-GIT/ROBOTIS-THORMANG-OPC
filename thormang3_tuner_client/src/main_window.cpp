/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman Jung */

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/thormang3_tuner_client/main_window.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace thormang3_tuner_client
{

using namespace Qt;

/*****************************************************************************
 ** Implementation [MainWindow]
 *****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent),
    qnode_(argc, argv),
    is_updating_(false)
{
  ui_.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui_.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));  // qApp is a global variable for the application

  setWindowIcon(QIcon(":/images/icon.png"));
  ui_.tab_manager->setCurrentIndex(0);  // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode_, SIGNAL(rosShutdown()), this, SLOT(close()));

  all_torque_on_ = false;

  // set list to make spinboxs
  spinBox_list_.push_back("goal");
  spinBox_list_.push_back("offset");
  spinBox_list_.push_back("mod");
  spinBox_list_.push_back("present");
  spinBox_list_.push_back("p_gain");
  spinBox_list_.push_back("i_gain");
  spinBox_list_.push_back("d_gain");
  spinBox_list_.push_back("velocity_p_gain");
  spinBox_list_.push_back("velocity_i_gain");
  spinBox_list_.push_back("velocity_d_gain");

  /****************************
   ** Connect
   ****************************/

  qRegisterMetaType<thormang3_tuning_module_msgs::JointOffsetPositionData>("thormang3_tuning_module_msgs::JointOffsetPositionData");
  QObject::connect(&qnode_, SIGNAL(updatePresentJointOffsetData(thormang3_tuning_module_msgs::JointOffsetPositionData)), this,
                   SLOT(updateJointOffsetSpinbox(thormang3_tuning_module_msgs::JointOffsetPositionData)));
  QObject::connect(&qnode_, SIGNAL(updateFT(bool, double)), this, SLOT(updateFT(bool, double)));
  QObject::connect(&qnode_, SIGNAL(updateIMU(double, double)), this, SLOT(updateIMU(double, double)));

  /*********************
   ** Logging
   **********************/
  ui_.view_logging->setModel(qnode_.loggingModel());
  QObject::connect(&qnode_, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  /****************************
   ** Connect
   ****************************/

  /*********************
   ** Auto Start
   **********************/
  qnode_.init();

  // make ui
  makeUI();
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
 ** Implementation [Slots]
 *****************************************************************************/

void MainWindow::on_save_offset_button_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "save_offset";

  qnode_.sendCommandMsg(msg);
}

void MainWindow::on_save_gain_button_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = "save_gain";

  qnode_.sendCommandMsg(msg);
}

void MainWindow::on_inipose_button_clicked(bool checck)
{
  std_msgs::String msg;
  msg.data = "ini_pose";

  qnode_.sendTuningPoseMsg(msg);
}

void MainWindow::on_tuning_pose_button_clicked(bool check)
{
  std_msgs::String msg;
  msg.data = ui_.tuning_pose_comboBox->currentText().toStdString();

  qnode_.sendTuningPoseMsg(msg);
}

void MainWindow::on_clear_button_clicked(bool check)
{
  // clear log window
  qnode_.clearLog();
}

void MainWindow::on_refresh_button_clicked(bool check)
{
  qnode_.getPresentJointOffsetData();
}

void MainWindow::clickedAllTorqueOnButton(QObject *button_group)
{
  all_torque_on_ = true;

  QButtonGroup* torque_button_group = qobject_cast<QButtonGroup*>(button_group);
  if (!torque_button_group)  // this is just a safety check
    return;

  QList<QAbstractButton *> torque_buttons = torque_button_group->buttons();
  for (int ix = 0; ix < torque_buttons.size(); ix++)
  {
    if (torque_buttons[ix]->isChecked() == false)
      torque_buttons[ix]->click();
  }

  qnode_.getPresentJointOffsetData(true);

  all_torque_on_ = false;
}

void MainWindow::clickedAllTorqueOffButton(QObject *button_group)
{
  QButtonGroup* torque_button_group = qobject_cast<QButtonGroup*>(button_group);
  if (!torque_button_group)  // this is just a safety check
    return;

  QList<QAbstractButton *> torque_buttons = torque_button_group->buttons();
  for (int ix = 0; ix < torque_buttons.size(); ix++)
  {
    if (torque_buttons[ix]->isChecked() == true)
      torque_buttons[ix]->click();
  }
}

void MainWindow::clickedTorqueCheckbox(QWidget *widget)
{
  QCheckBox* checkBox = qobject_cast<QCheckBox*>(widget);
  if (!checkBox)  // this is just a safety check
    return;

  std::string joint_name = checkBox->text().toStdString();
  bool is_on = checkBox->isChecked();

  QList<QAbstractSpinBox *> spinbox_list = joint_spinbox_map_[joint_name];

  for (int ix = 0; ix < spinbox_list.size(); ix++)
  {
    spinbox_list[ix]->setEnabled(is_on);
  }

  publishTorqueMsgs(joint_name, is_on);
}

void MainWindow::publishTorqueMsgs(std::string &joint_name, bool torque_on)
{
  thormang3_tuning_module_msgs::JointTorqueOnOffArray torque_array_msg;
  thormang3_tuning_module_msgs::JointTorqueOnOff torque_msg;

  torque_msg.joint_name = joint_name;
  torque_msg.torque_enable = torque_on;

  torque_array_msg.torque_enable_data.push_back(torque_msg);

  qnode_.sendTorqueEnableMsg(torque_array_msg);

  if (all_torque_on_ == false)
    qnode_.getPresentJointOffsetData(true);
}

void MainWindow::changedOffsetSpinBoxValue(QString q_joint_name)
{
  if (qnode_.isRefresh() == true || is_updating_ == true)
    return;

  thormang3_tuning_module_msgs::JointOffsetData msg;
  std::string joint_name = q_joint_name.toStdString();

  QList<QAbstractSpinBox *> spinbox_list = joint_spinbox_map_[joint_name];
  QDoubleSpinBox *mod_spinBox;

  msg.joint_name = joint_name;

  for (int ix = 0; ix < spinbox_list.size(); ix++)
  {
    if (spinbox_list[ix]->whatsThis().toStdString() == "goal")
    {
      QDoubleSpinBox* spinBox = qobject_cast<QDoubleSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      msg.goal_value = spinBox->value() * M_PI / 180.0;
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "offset")
    {
      QDoubleSpinBox* spinBox = qobject_cast<QDoubleSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      msg.offset_value = spinBox->value() * M_PI / 180.0;
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "mod")
    {
      mod_spinBox = qobject_cast<QDoubleSpinBox*>(spinbox_list[ix]);
    }
  }

  if (mod_spinBox)  // this is just a safety check
    mod_spinBox->setValue((msg.goal_value + msg.offset_value) * 180.0 / M_PI);

  qnode_.sendJointOffsetDataMsg(msg);
}

void MainWindow::changedGainSpinBoxValue(QString q_joint_name)
{
  if (qnode_.isRefresh() == true || is_updating_ == true)
    return;

  thormang3_tuning_module_msgs::JointOffsetData msg;
  std::string joint_name = q_joint_name.toStdString();

  QList<QAbstractSpinBox *> spinbox_list = joint_spinbox_map_[joint_name];
  QDoubleSpinBox *mod_spinBox;

  msg.joint_name = joint_name;

  for (int ix = 0; ix < spinbox_list.size(); ix++)
  {
    if (spinbox_list[ix]->whatsThis().toStdString() == "p_gain")
    {
      QSpinBox* spinBox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      if(spinBox->isEnabled() == true)
        msg.p_gain = spinBox->value();
      else
        msg.p_gain = NONE_GAIN;
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "i_gain")
    {
      QSpinBox* spinBox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      if(spinBox->isEnabled() == true)
        msg.i_gain = spinBox->value();
      else
        msg.i_gain = NONE_GAIN;
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "d_gain")
    {
      QSpinBox* spinBox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      if(spinBox->isEnabled() == true)
        msg.d_gain = spinBox->value();
      else
        msg.d_gain = NONE_GAIN;
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "velocity_p_gain")
    {
      QSpinBox* spinBox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      if(spinBox->isEnabled() == true)
        msg.velocity_p_gain = spinBox->value();
      else
        msg.velocity_p_gain = NONE_GAIN;
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "velocity_i_gain")
    {
      QSpinBox* spinBox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      if(spinBox->isEnabled() == true)
        msg.velocity_i_gain = spinBox->value();
      else
        msg.velocity_i_gain = NONE_GAIN;
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "velocity_d_gain")
    {
      QSpinBox* spinBox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      if(spinBox->isEnabled() == true)
        msg.velocity_d_gain = spinBox->value();
      else
        msg.velocity_d_gain = NONE_GAIN;
    }
  }

  qnode_.sendJointGainDataMsg(msg);
}

void MainWindow::updateJointOffsetSpinbox(thormang3_tuning_module_msgs::JointOffsetPositionData msg)
{
  is_updating_ = true;

  std::string joint_name = msg.joint_name;

  QList<QAbstractSpinBox *> spinbox_list = joint_spinbox_map_[joint_name];

  for (int ix = 0; ix < spinbox_list.size(); ix++)
  {
    if (spinbox_list[ix]->whatsThis().toStdString() == "goal")
    {
      QDoubleSpinBox* spinBox = qobject_cast<QDoubleSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      spinBox->setValue(msg.goal_value * 180.0 / M_PI);
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "offset")
    {
      QDoubleSpinBox* spinBox = qobject_cast<QDoubleSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      spinBox->setValue(msg.offset_value * 180.0 / M_PI);
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "mod")
    {
      QDoubleSpinBox* spinBox = qobject_cast<QDoubleSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      spinBox->setValue(msg.goal_value * 180.0 / M_PI + msg.offset_value * 180.0 / M_PI);
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "present")
    {
      QDoubleSpinBox* spinBox = qobject_cast<QDoubleSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      spinBox->setValue(msg.present_value * 180.0 / M_PI);
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "p_gain")
    {
      QSpinBox* spinBox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      if(msg.p_gain == NONE_GAIN)
      {
        spinBox->setEnabled(false);
      }
      else
        spinBox->setValue(msg.p_gain);
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "i_gain")
    {
      QSpinBox* spinBox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      if(msg.i_gain == NONE_GAIN)
      {
        spinBox->setEnabled(false);
      }
      else
        spinBox->setValue(msg.i_gain);
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "d_gain")
    {
      QSpinBox* spinBox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      if(msg.d_gain == NONE_GAIN)
      {
        spinBox->setEnabled(false);
      }
      else
        spinBox->setValue(msg.d_gain);
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "velocity_p_gain")
    {
      QSpinBox* spinBox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      if(msg.velocity_p_gain == NONE_GAIN)
      {
        spinBox->setEnabled(false);
      }
      else
        spinBox->setValue(msg.velocity_p_gain);
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "velocity_i_gain")
    {
      QSpinBox* spinBox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      if(msg.velocity_i_gain == NONE_GAIN)
      {
        spinBox->setEnabled(false);
      }
      else
        spinBox->setValue(msg.velocity_i_gain);
    }
    else if (spinbox_list[ix]->whatsThis().toStdString() == "velocity_d_gain")
    {
      QSpinBox* spinBox = qobject_cast<QSpinBox*>(spinbox_list[ix]);
      if (!spinBox)  // this is just a safety check
        continue;

      if(msg.velocity_d_gain == NONE_GAIN)
      {
        spinBox->setEnabled(false);
      }
      else
        spinBox->setValue(msg.velocity_d_gain);
    }
  }

  is_updating_ = false;
}

/*****************************************************************************
 ** Implemenation [Slots][manually connected]
 *****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView()
{
  ui_.view_logging->scrollToBottom();
}

void MainWindow::updateFT(bool is_right, double ft_value)
{
  if(is_right == true)
    ui_.doubleSpinBox_force_right->setValue(ft_value);
  else
    ui_.doubleSpinBox_force_left->setValue(ft_value);
}

void MainWindow::updateIMU(double roll, double pitch)
{
  ui_.doubleSpinBox_imu_roll->setValue(roll);
  ui_.doubleSpinBox_imu_pitch->setValue(pitch);
}

void MainWindow::makeUI()
{
  makeTabUI(ui_.right_arm_group, qnode_.right_arm_offset_group_);
  makeTabUI(ui_.left_arm_group, qnode_.left_arm_offset_group_);
  makeTabUI(ui_.leg_group, qnode_.legs_offset_group_);
  makeTabUI(ui_.body_group, qnode_.body_offset_group_);
}

//void MainWindow::makeTabUI(QGroupBox *joint_widget, QGroupBox *torque_widget, QButtonGroup *button_group,
//                           std::map<int, std::string> &offset_group)
void MainWindow::makeTabUI(QGroupBox *joint_widget, std::map<int, std::string> &offset_group)
{
  //  QSignalMapper *torque_checkbox_signalMapper = new QSignalMapper(this);

  QGridLayout *grid_layout = (QGridLayout *) joint_widget->layout();
  //  QGridLayout *torque_layout = (QGridLayout *) torque_widget->layout();

  //  button_group = new QButtonGroup();
  //  button_group->setExclusive(false);

  int num_row = 3;
  //  int torque_checkbox_index = 0;
  //  int torque_row = 1;
  //  int torque_col = 0;
  for (std::map<int, std::string>::iterator map_it = offset_group.begin(); map_it != offset_group.end(); ++map_it)
  {
    QSignalMapper *spinbox_offset_signalMapper = new QSignalMapper(this);
    QSignalMapper *spinbox_gain_signalMapper = new QSignalMapper(this);
    QList<QAbstractSpinBox *> spinbox_list;

    // spin_box
    int num_col = 0;
    int spinbox_size = 1;
    std::string joint_name = map_it->second;
    QString q_joint_name = QString::fromStdString(joint_name);

    // label
    QLabel *joint_label = new QLabel(q_joint_name);
    grid_layout->addWidget(joint_label, num_row, num_col++, 1, spinbox_size);

    // double spin box : goal, offset, mod, present
    for (int ix = 0; ix < 4; ix++)
    {
      QDoubleSpinBox *spin_box = new QDoubleSpinBox();
      spin_box->setWhatsThis(tr(spinBox_list_[ix].c_str()));
      spin_box->setMinimum(-360);
      spin_box->setMaximum(360);
      spin_box->setSingleStep(0.05);

      switch (ix)
      {
      case 2:
      case 3:
        spin_box->setReadOnly(true);
        break;

      default:
        spinbox_offset_signalMapper->setMapping(spin_box, q_joint_name);
        QObject::connect(spin_box, SIGNAL(valueChanged(QString)), spinbox_offset_signalMapper, SLOT(map()));
        break;
      }

      grid_layout->addWidget(spin_box, num_row, num_col++, 1, spinbox_size);

      spinbox_list.append(spin_box);
    }

    // spin box : p gain, i gain, d gain
    // spin box : velocity p gain, i gain, d gain
    for (int ix = 0; ix < 6; ix++)
    {
      QSpinBox *spin_box = new QSpinBox();
      spin_box->setWhatsThis(tr(spinBox_list_[ix + 4].c_str()));
      spin_box->setMinimum(0);
      spin_box->setMaximum(65535);
      spin_box->setSingleStep(10);

      switch (ix)
      {
      case 0:
        spin_box->setValue(0);
        break;

      case 1:
        spin_box->setValue(0);
        break;

      case 2:
        spin_box->setValue(0);
        break;

      default:
        spin_box->setValue(0);
        //spin_box->setReadOnly(true);
        break;
      }

      spinbox_gain_signalMapper->setMapping(spin_box, q_joint_name);
      QObject::connect(spin_box, SIGNAL(valueChanged(QString)), spinbox_gain_signalMapper, SLOT(map()));

      grid_layout->addWidget(spin_box, num_row, num_col++, 1, spinbox_size);

      spinbox_list.append(spin_box);
    }

    // spinbox
    joint_spinbox_map_[joint_name] = spinbox_list;
    QObject::connect(spinbox_offset_signalMapper, SIGNAL(mapped(QString)), this, SLOT(changedOffsetSpinBoxValue(QString)));
    QObject::connect(spinbox_gain_signalMapper, SIGNAL(mapped(QString)), this, SLOT(changedGainSpinBoxValue(QString)));

    num_row += 1;

    // torque checkbox
    //    torque_row = torque_checkbox_index / 6;
    //    torque_col = torque_checkbox_index % 6;

    //    QCheckBox *torque_check_box = new QCheckBox(q_joint_name);
    //    torque_check_box->setChecked(true);
    //    torque_layout->addWidget(torque_check_box, torque_row, torque_col, 1, spinbox_size);
    //    button_group->addButton(torque_check_box);

    //    torque_checkbox_signalMapper->setMapping(torque_check_box, torque_check_box);
    //    QObject::connect(torque_check_box, SIGNAL(clicked()), torque_checkbox_signalMapper, SLOT(map()));

    //    torque_checkbox_index += 1;
  }

  // all torque on
  //  QSignalMapper *torque_on_signalMapper = new QSignalMapper(this);
  //  QPushButton *torque_on_button = new QPushButton(tr("All torque ON"));
  //  torque_layout->addWidget(torque_on_button, torque_row + 1, 4, 1, 1);
  //  torque_on_signalMapper->setMapping(torque_on_button, button_group);
  //  QObject::connect(torque_on_button, SIGNAL(clicked()), torque_on_signalMapper, SLOT(map()));
  //  QObject::connect(torque_on_signalMapper, SIGNAL(mapped(QObject*)), this, SLOT(clickedAllTorqueOnButton(QObject*)));

  // all torque off
  //  QSignalMapper *torque_off_signalMapper = new QSignalMapper(this);
  //  QPushButton *torque_off_button = new QPushButton(tr("All torque OFF"));
  //  torque_layout->addWidget(torque_off_button, torque_row + 1, 5, 1, 1);
  //  torque_off_signalMapper->setMapping(torque_off_button, button_group);
  //  QObject::connect(torque_off_button, SIGNAL(clicked()), torque_off_signalMapper, SLOT(map()));
  //  QObject::connect(torque_off_signalMapper, SIGNAL(mapped(QObject*)), this, SLOT(clickedAllTorqueOffButton(QObject*)));

  //  QObject::connect(torque_checkbox_signalMapper, SIGNAL(mapped(QWidget*)), this, SLOT(clickedTorqueCheckbox(QWidget*)));
}

/*****************************************************************************
 ** Implementation [Menu]
 *****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this, tr("About ..."), tr("<h2>Robotis Offset Tuner Clinet 0.10</h2><p>Copyright ROBOTIS</p>"));
}

/*****************************************************************************
 ** Implementation [Configuration]
 *****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

}  // namespace thormang3_tuner_client

