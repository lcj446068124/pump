#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <qfiledialog.h>
using Qt::endl;
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("三元组写入工具");
    this->setFixedSize(811,512);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_RomSelButton_clicked()
{
    this->RomPath = QFileDialog::getOpenFileName(this, "选择Rom文件", "C:/","*.bin");
    this->ui->RomSelLineEdit->setText(this->RomPath);


}

void MainWindow::on_AliSelButton_clicked()
{
    this->AliPath = QFileDialog::getOpenFileName(this, "选择三元组文件", "C:/","*.csv");
    this->ui->AliSelLineEdit->setText(this->AliPath);
}


QByteArray basicoutput(1024 * 62,0);

void MainWindow::on_Start_clicked()
{
    ui->Start->setEnabled(false);
    this->ui->resultOutput->append("***************开始生成！***************\n");
    QFile RomBinary(this->RomPath);
    QFile AliText(this->AliPath);
    outPutFilePath = "D:/";
    if(!RomBinary.open(QIODevice::ReadOnly) || !AliText.open(QIODevice::ReadOnly))
    {
        qDebug()<<"Can't open the file!"<<endl;
    }
    qDebug()<<"RomBinary size: "<<RomBinary.size()<<endl;
    qDebug()<<"AliText size: "<<AliText.size()<<endl;
    int romPtr = 0;
    for(romPtr = 0;romPtr < RomBinary.size() && romPtr < 1024 * 62;romPtr++){
        uint8_t sub;
        RomBinary.read((char *)&sub,sizeof(sub));
        //qDebug()<<"i: "<<romPtr<<"sub"<<sub<<endl;
        basicoutput[romPtr] = sub;
    }
    while (romPtr < 1024 * 62) {
        basicoutput[romPtr++] = 255;
    }

    qDebug()<<"output.size(): "<<basicoutput.size()<<endl;
    QByteArray line = AliText.readLine().trimmed();//文件头
    while (!AliText.atEnd())
    {
       line = AliText.readLine().trimmed();
       QList<QByteArray> localSplit = line.split(',');
       qDebug() << localSplit.data();
       QByteArray output(basicoutput);
       QString outPutFileName(localSplit[0]);
       localSplit[0].append('\0');
       output.replace(1024 * 60,localSplit[0].size(),localSplit[0]);
       localSplit[1].append('\0');
       output.replace(1024 * 60 + 64 * 1,localSplit[1].size(),localSplit[1]);
       localSplit[2].append('\0');
       output.replace(1024 * 60 + 64 * 2,localSplit[2].size(),localSplit[2]);
       outPutFileName.append(".bin");
       QFile outPutFile(outPutFilePath + outPutFileName);
       outPutFile.open(QIODevice::WriteOnly);
       outPutFile.write(output);
       outPutFile.close();
       this->ui->resultOutput->append(outPutFilePath + outPutFileName + " create success!\n");
     }

    AliText.close();
    RomBinary.close();
    ui->Start->setEnabled(true);
    this->ui->resultOutput->append("***************生成结束！***************\n");
}
