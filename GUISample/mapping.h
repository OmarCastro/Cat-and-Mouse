#ifndef MAPPING_H
#define MAPPING_H

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <vector>
#include "tactics.h"
using std::vector;

class MappingScene : public QGraphicsScene
{
    Q_OBJECT
public:
    MappingScene(RobotMap* map, QObject *parent = 0);
    void redrawMap();
signals:

public slots:

private:
    RobotMap* map;
    vector<QGraphicsRectItem*> mapblock;

};

class Mapping : public QGraphicsView
{
    Q_OBJECT
public:
    Mapping(RobotMap* map, char* rob_name);
    
signals:
    
public slots:
    void redrawMap(){scene->redrawMap();}
private:
    MappingScene* scene;
    
};

#endif // MAPPING_H
