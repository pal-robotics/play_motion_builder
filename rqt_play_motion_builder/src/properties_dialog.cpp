#include <rqt_play_motion_builder/properties_dialog.h>

#include <QFileDialog>

namespace pal
{
MotionProperties::MotionProperties(QWidget *parent) : QDialog(parent)
{
  ui_.setupUi(this);
  QLabel *out_label = new QLabel();
  out_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  QVBoxLayout *layout = new QVBoxLayout();
  out_label->setLayout(layout);
  ui_.scrollArea->setWidget(out_label);
  ui_.scrollArea->setWidgetResizable(true);

  connect(ui_.cancel_btn_, SIGNAL(pressed()), this, SLOT(onCancel()));
  connect(ui_.build_yaml_btn_, SIGNAL(pressed()), this, SLOT(onSaveYAML()));
  connect(ui_.yaml_name_, SIGNAL(textChanged(QString)), this, SLOT(onTextChanged(QString)));
}

void MotionProperties::init(ros::NodeHandle &nh)
{
  store_client_ = nh.serviceClient<play_motion_builder_msgs::StoreMotion>(
      "/play_motion_builder_node/store_motion");
}

void MotionProperties::reset()
{
  ui_.yaml_name_->setText("");
  ui_.name_->setText("");
  ui_.usage_->setText("");
  ui_.description_->setText("");
}

void MotionProperties::loadYamlName(const std::string &yaml_name)
{
  ui_.yaml_name_->setText(QString::fromStdString(yaml_name));
}

void MotionProperties::loadMeta(const std::string &name, const std::string &usage,
                                const std::string &description)
{
  ui_.name_->setText(QString::fromStdString(name));
  ui_.usage_->setText(QString::fromStdString(usage));
  ui_.description_->setText(QString::fromStdString(description));
}

void MotionProperties::onCancel()
{
  this->close();
}

void MotionProperties::onTextChanged(const QString &text)
{
  ui_.build_yaml_btn_->setEnabled(!text.isEmpty());
}

void MotionProperties::onSaveYAML()
{
  QString file = QFileDialog::getSaveFileName(this);
  ROS_INFO_STREAM("Directory: " << file.toStdString());

  play_motion_builder_msgs::StoreMotion sm;
  sm.request.file_path = file.toStdString();
  sm.request.ros_name = ui_.yaml_name_->text().toStdString();
  sm.request.meta.name = ui_.name_->text().toStdString();
  sm.request.meta.usage = ui_.usage_->text().toStdString();
  sm.request.meta.description = ui_.description_->text().toStdString();

  if (store_client_.call(sm))
  {
    if (sm.response.ok)
    {
      emit motionStored(ui_.yaml_name_->text());
    }
    else
    {
      ROS_ERROR_STREAM("Error on storing: " << sm.response.message);
    }
  }
  else
  {
    ROS_ERROR_STREAM("Couldn't connect with the store service");
  }
}

}  // namespace pal
