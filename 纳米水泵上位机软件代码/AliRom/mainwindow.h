#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_RomSelButton_clicked();

    void on_AliSelButton_clicked();

    void on_Start_clicked();

private:
    Ui::MainWindow *ui;
    QString RomPath;
    QString AliPath;
    QString outPutFilePath;
};
#endif // MAINWINDOW_H
