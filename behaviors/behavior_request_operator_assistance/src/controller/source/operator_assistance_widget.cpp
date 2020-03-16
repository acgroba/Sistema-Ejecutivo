
/*!*******************************************************************************************
 *  \file       operator_assistance_widget.cpp
 *  \brief      OperatorAssistanceWidget implementation file.
 *  \details    This file implements the OperatorAssistanceWidget class.
 *  \authors    Abraham Carrera.
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
 *              All rights reserved
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include <operator_assistance_widget.h>
#include <nodelet/nodelet.h>

OperatorAssistanceWidget::OperatorAssistanceWidget(QWidget *parent)
    : QWidget(parent), ui(new Ui::OperatorAssistanceWidget)
{
  ui->setupUi(this);
  layout = ui->gridLayout;
  button_layout = ui->button_layout;
  text_edit = ui->text_edit;
}

OperatorAssistanceWidget::OperatorAssistanceWidget(std::string message, std::string belief_predicate_name,
                                                  BehaviorRequestOperatorAssistance *behavior)
    : QWidget(0), ui(new Ui::OperatorAssistanceWidget)
{
  ui->setupUi(this);
  layout = ui->gridLayout;
  button_layout = ui->button_layout;
  text_edit = ui->text_edit;
  confirm =  ui->accept;
  connect(confirm, SIGNAL(clicked()), this, SLOT(confirmElection()));

  confirm->setEnabled(false);
  this->behavior = behavior;
  ui->label->setText(QString::fromUtf8("Question: "));
  text_edit->setPlainText(QString::fromUtf8((message).c_str()));
  this->belief_predicate_name = belief_predicate_name;
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,sizeof(buffer),"%H:%M:%S",timeinfo);
  std::string str(buffer);
  ui->time->setText(QString::fromUtf8("Time: ")+QString::fromUtf8(str.c_str()));
}

OperatorAssistanceWidget::~OperatorAssistanceWidget()
{
  delete mapper;
  delete ui;
}

void OperatorAssistanceWidget::confirmElection()
{

  this->QWidget::close();
  std::string selected_option;
  for (int i = 0; i < checkboxes.size(); i++)
  {
    if (checkboxes[i]->isChecked())
      selected_option = checkboxes[i]->text().toStdString();
  }
  std::string belief_to_add;
  belief_to_add = belief_predicate_name + "(" + selected_option + ")";
  behavior->addBelief(belief_to_add);

}

void OperatorAssistanceWidget::setUpOptions(std::vector<std::string> options)
{
  if (options.size() != 0)
  {
    int row = 1;
    for (int i = 0; i < options.size(); i++)
    {
      QCheckBox *new_option = new QCheckBox(QString::fromStdString(options[i]), this);
      QObject::connect(new_option, SIGNAL(stateChanged(int)), this, SLOT(checkBoxStateChanged(int)));
      checkboxes.push_back(new_option);
      button_layout->addWidget(new_option, row, 0, 1, 2);
      row++;
    }
  }
}

void OperatorAssistanceWidget::checkBoxStateChanged(int state)
{
  QCheckBox *aux_checkbox = (QCheckBox *)QObject::sender();
  for (int i = 0; i < checkboxes.size(); i++)
  {
    if (state == 0)
    {
      break;
    }
    if (state == 2)
    {
      if (checkboxes[i] != aux_checkbox)
      {
        checkboxes[i]->setChecked(false);
      }
      else
      {
        checkboxes[i]->setChecked(true);
      }
    }
  }
  confirm->setEnabled(false);
  for (int i = 0; i < checkboxes.size(); i++)
  {
    if (checkboxes[i]->isChecked())
      confirm->setEnabled(true);
  }
}
