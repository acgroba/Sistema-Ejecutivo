/*!*********************************************************************************
 *  \file       inform_operator_widget.h
 *  \brief      InformOperatorWidget definition file.
 *  \details    This file contains the InformOperatorWidget declaration. To obtain more information about
 *              it's definition consult the inform_operator_widget.cpp file.
 *  \authors    Abraham Carrera Groba.
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

#ifndef INFORM_OPERATOR_WIDGET_H
#define INFORM_OPERATOR_WIDGET_H

#include "ui_inform_operator_widget.h"
#include <QApplication>
#include <QButtonGroup>
#include <QCheckBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalMapper>
#include <QVBoxLayout>
#include <QWidget>
#include <ctime>

// Forward declaration
class BehaviorInformOperator;
#include <behavior_inform_operator.h>

namespace Ui
{
class InformOperatorWidget;
}

class InformOperatorWidget : public QWidget
{
  Q_OBJECT
public:
  explicit InformOperatorWidget(QWidget *parent = 0);
  InformOperatorWidget(std::string message, BehaviorInformOperator *behavior);
  ~InformOperatorWidget();

private:
  Ui::InformOperatorWidget *ui;
  QGridLayout *layout;
  QGridLayout *button_layout;
  QPushButton *confirm;
  QPlainTextEdit *text_edit;
  QSignalMapper *mapper;

  BehaviorInformOperator *behavior;
  std::vector<QPushButton *> button_list;

public Q_SLOTS:
  void confirmation();
};

#endif
