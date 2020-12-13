#ifndef INDEX_H
#define INDEX_H

#include <QThread>
#include <QWidget>

namespace Ui {
class Index;
}

class Index : public QWidget
{
    Q_OBJECT

public:
    explicit Index(QWidget *parent = 0);
    ~Index();  

private:
    Ui::Index *ui;
    QList<QThread *> thread_list;

signals:
    void stop();
};

#endif // INDEX_H
