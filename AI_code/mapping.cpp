#include "mapping.h"

const int mapsize = RobotMap::mapsize;

MappingScene::MappingScene(RobotMap* map, QObject *parent)
    :QGraphicsScene(parent)
    ,map(map){

    mapblock.resize(mapsize*mapsize);
    for(int i=0; i < mapsize;++i){
        for(int j=0;j < mapsize; j++){
            QGraphicsRectItem *rect = new QGraphicsRectItem(5*i,5*j,5,5,0,this);
            rect->setBrush(Qt::gray);
            rect->setPen(QPen(Qt::gray));
            mapblock[mapsize*i+j] = rect;

        }
    }
}

void MappingScene::redrawMap(){
    for(int y=0; y < mapsize;++y){
        for(int x=0;x < mapsize; x++){
            double value = map->getArrayValue(x,y);
            QGraphicsRectItem* rect = mapblock[mapsize*y+x];
            if(value > 0.5){
                rect->setBrush(Qt::black);
                rect->setPen(QPen(Qt::black));

            } else if (value < 0.4){

                rect->setBrush(Qt::red);
                rect->setPen(QPen(Qt::red));

            } else {
                rect->setBrush(Qt::gray);
                rect->setPen(QPen(Qt::gray));
            }
        }
    }

}

Mapping::Mapping(RobotMap *map, char *rob_name)
{
    setWindowTitle(rob_name);
    scene =new MappingScene(map);
    setScene(scene);
}
