/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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
 *
 */

#include <gazebo/common/CommonIface.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/MainWindow.hh>

class GazeboRosEolGui : public gazebo::GUIPlugin
{
  Q_OBJECT

  /// \brief Constructor
  public: GazeboRosEolGui () : GUIPlugin()
  {
    // Gazebo 11.15.0 and later contain the EOL warning in the core, so there's
    // no need to add the notice here.
    if (GAZEBO_MINOR_VERSION < 15)
    {
      auto mainWindow = gazebo::gui::get_main_window();
      QWidget *menuFrame = mainWindow->menuWidget();
      QHBoxLayout *menuLayout =
          dynamic_cast<QHBoxLayout *>(menuFrame->layout());
      if (!menuLayout)
      {
        gzerr << "GazeboRosEolGui: Could not find menu layout. Not adding EOL "
                 "notice\n";
        return;
      }

      const char *ignoreEol =
          gazebo::common::getEnv("GAZEBO_SUPPRESS_EOL_WARNING");
      if (ignoreEol == nullptr || strncmp(ignoreEol, "1", 1) != 0)
      {
        // Add Gazebo classic EOL notice label
        QList<QMenuBar *> menuBars = mainWindow->findChildren<QMenuBar *>();
        auto eolNotice = new QLabel(menuBars[0]);
        // It's important that the text in the href does not contain new lines.
        // Otherwise, the resulting url will be invalid.
        eolNotice->setText(R"(<font color='#ff9966'>
          This version of Gazebo reaches end-of-life in January 2025.
          Consider <a style='color: #ffcc00'
          href='https://gazebosim.org/docs/latest/gazebo_classic_migration/?utm_source=gazebo_ros_pkgs'
          >migrating to the new Gazebo</a></font>)");
        eolNotice->setOpenExternalLinks(true);
        menuLayout->addStretch(1);
        menuLayout->addWidget(eolNotice);
        menuLayout->addStretch(4);
      }

      this->move(0, 0);
      this->resize(0, 0);
    }
  }
};

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GazeboRosEolGui)

#include "gazebo_ros_eol_gui.moc"
