#include <iostream>

#include "scenario.h"

#include "work/closedsubdivisioncurve.h"
// hidmanager
#include "hidmanager/defaulthidmanager.h"

// gmlib
#include <scene/light/gmpointlight.h>
#include <scene/sceneobjects/gmpathtrack.h>
#include <scene/sceneobjects/gmpathtrackarrows.h>

// qt
#include <QQuickItem>

template <typename T>
inline std::ostream &operator<<(std::ostream &out, const std::vector<T> &v)
{
  out << v.size() << std::endl;
  for (uint i = 0; i < v.size(); i++)
    out << " " << v[i];
  out << std::endl;
  return out;
}

void Scenario::initializeScenario()
{

  // Insert a light
  GMlib::Point<GLfloat, 3> init_light_pos(2.0, 4.0, 10);
  GMlib::PointLight *light = new GMlib::PointLight(GMlib::GMcolor::white(), GMlib::GMcolor::white(),
                                                   GMlib::GMcolor::white(), init_light_pos);
  light->setAttenuation(0.8f, 0.002f, 0.0008f);
  this->scene()->insertLight(light, false);

  // Insert Sun
  this->scene()->insertSun();

  // Default camera parameters
  int init_viewport_size = 600;
  GMlib::Point<float, 3> init_cam_pos(0.0f, 0.0f, 0.0f);
  GMlib::Vector<float, 3> init_cam_dir(0.0f, 1.0f, 0.0f);
  GMlib::Vector<float, 3> init_cam_up(1.0f, 0.0f, 0.0f);

  // Projection cam
  auto proj_rcpair = createRCPair("Projection");
  proj_rcpair.camera->set(init_cam_pos, init_cam_dir, init_cam_up);
  proj_rcpair.camera->setCuttingPlanes(1.0f, 8000.0f);
  proj_rcpair.camera->rotateGlobal(GMlib::Angle(-45), GMlib::Vector<float, 3>(1.0f, 0.0f, 0.0f));
  proj_rcpair.camera->translateGlobal(GMlib::Vector<float, 3>(0.0f, -20.0f, 20.0f));
  scene()->insertCamera(proj_rcpair.camera.get());
  proj_rcpair.renderer->reshape(GMlib::Vector<int, 2>(init_viewport_size, init_viewport_size));

  /***************************************************************************
   *                                                                         *
   * Standar example, including path track and path track arrows             *
   *                                                                         *
   ***************************************************************************/

/*   GMlib::Material mm(GMlib::GMmaterial::polishedBronze());
  mm.set(45.0);

  auto ptom = new TestTorus(1.0f, 0.4f, 0.6f);
  ptom->toggleDefaultVisualizer();
  ptom->sample(60, 60, 1, 1);
  this->scene()->insert(ptom);
  auto ptrack = new GMlib::PathTrack();
  ptrack->setLineWidth(2);
  ptom->insert(ptrack);
  auto ptrack2 = new GMlib::PathTrackArrows();
  ptrack2->setArrowLength(2);
  ptom->insert(ptrack2); */


  // First rectangle shape (centered vertically at y = 0)
  {
    float offsetY = 0.0f;
    GMlib::DVector<GMlib::Vector<float, 3>> rectPoints(4, GMlib::Vector<float, 3>(0.0f, 0.0f, 0.0f));
    rectPoints[0] = GMlib::Vector<float, 3>(-1.0f, offsetY - 1.0f, 0.0f);
    rectPoints[1] = GMlib::Vector<float, 3>( 1.0f, offsetY - 1.0f, 0.0f);
    rectPoints[2] = GMlib::Vector<float, 3>( 1.0f, offsetY + 1.0f, 0.0f);
    rectPoints[3] = GMlib::Vector<float, 3>(-1.0f, offsetY + 1.0f, 0.0f);

    auto rect1 = new ClosedSubdivisionCurve(rectPoints, 4);
    rect1->toggleDefaultVisualizer();
    rect1->sample(500);
    this->scene()->insert(rect1);
  }

  // Second rectangle shape (shifted up by 2 along the y axis)
  {
    float offsetY = 3.0f;
    GMlib::DVector<GMlib::Vector<float, 3>> rectPoints(4, GMlib::Vector<float, 3>(0.0f, 0.0f, 0.0f));
    rectPoints[0] = GMlib::Vector<float, 3>(-1.0f, offsetY - 1.0f, 0.0f);
    rectPoints[1] = GMlib::Vector<float, 3>( 1.0f, offsetY - 1.0f, 0.0f);
    rectPoints[2] = GMlib::Vector<float, 3>( 1.0f, offsetY + 1.0f, 0.0f);
    rectPoints[3] = GMlib::Vector<float, 3>(-1.0f, offsetY + 1.0f, 0.0f);

    auto rect2 = new ClosedSubdivisionCurve(rectPoints, 3);
    rect2->toggleDefaultVisualizer();
    rect2->sample(500);
    this->scene()->insert(rect2);
  }

  // Third rectangle shape (shifted up by 4 along the y axis)
  {
    float offsetY = 6.0f;
    GMlib::DVector<GMlib::Vector<float, 3>> rectPoints(4, GMlib::Vector<float, 3>(0.0f, 0.0f, 0.0f));
    rectPoints[0] = GMlib::Vector<float, 3>(-1.0f, offsetY - 1.0f, 0.0f);
    rectPoints[1] = GMlib::Vector<float, 3>( 1.0f, offsetY - 1.0f, 0.0f);
    rectPoints[2] = GMlib::Vector<float, 3>( 1.0f, offsetY + 1.0f, 0.0f);
    rectPoints[3] = GMlib::Vector<float, 3>(-1.0f, offsetY + 1.0f, 0.0f);

    auto rect3 = new ClosedSubdivisionCurve(rectPoints, 2);
    rect3->toggleDefaultVisualizer();
    rect3->sample(500);
    this->scene()->insert(rect3);
  }
}

void Scenario::cleanupScenario()
{
}

void Scenario::callDefferedGL()
{

  GMlib::Array<const GMlib::SceneObject *> e_obj;
  this->scene()->getEditedObjects(e_obj);

  for (int i = 0; i < e_obj.getSize(); i++)
    if (e_obj(i)->isVisible())
      e_obj[i]->replot();
}
