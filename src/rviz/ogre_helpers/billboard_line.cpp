/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "billboard_line.h"

#include <OgreBillboardChain.h>
#include <OgreMaterialManager.h>
#include <OgreQuaternion.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreVector3.h>

#include <sstream>

#include <ros/assert.h>

#define MAX_ELEMENTS (65536/4)

namespace rviz
{

using Ogre::Camera;
using Ogre::HardwareVertexBufferSharedPtr;
using Ogre::HardwareBuffer;
using Ogre::Vector3;
using Ogre::uint16;
using Ogre::RGBA;
using Ogre::Root;
using Ogre::String;
using Ogre::StringConverter;
using Ogre::NameValuePairList;
using Ogre::MovableObjectFactory;
class CustomOgreBillboardChain : public Ogre::BillboardChain
{
public:
  using Ogre::BillboardChain::BillboardChain;

  const String&
  getMovableType(void) const
  {
      return CustomOgreBillboardChainFactory::FACTORY_TYPE_NAME;
  }

  void updateVertexBuffer(Camera* cam)
  {
      printf("here\n");
      setupBuffers();
      HardwareVertexBufferSharedPtr pBuffer =
         mVertexData->vertexBufferBinding->getBuffer(0);
      void* pBufferStart = pBuffer->lock(HardwareBuffer::HBL_DISCARD);

      const Vector3& camPos = cam->getDerivedPosition();
      Vector3 eyePos = mParentNode->_getDerivedOrientation().Inverse() *
         (camPos - mParentNode->_getDerivedPosition()) / mParentNode->_getDerivedScale();



      Vector3 chainTangent;
      for (ChainSegmentList::iterator segi = mChainSegmentList.begin();
         segi != mChainSegmentList.end(); ++segi)
      {
         ChainSegment& seg = *segi;

         // Skip 0 or 1 element segment counts
         if (seg.head != SEGMENT_EMPTY && seg.head != seg.tail)
         {
            size_t laste = seg.head;

            // mkultra333 create tangent based on normal of triangle made from eye pos, first elem pos and last elem pos
            Vector3 CamToElemPosHead=mChainElementList[seg.head + seg.start].position-eyePos ;
            Vector3 CamToElemPosTail=mChainElementList[seg.tail + seg.start].position-eyePos ;
            Vector3 AltTangent=CamToElemPosHead.crossProduct(CamToElemPosTail) ;
            AltTangent.normalise() ;
            AltTangent*=0.5 ; // halve it, saves doing it over and over in the chain


            for (size_t e = seg.head; ; ++e) // until break
            {
               // Wrap forwards
               if (e == mMaxElementsPerChain)
                  e = 0;

               Element& elem = mChainElementList[e + seg.start];
               assert (((e + seg.start) * 2) < 65536 && "Too many elements!");
               uint16 baseIdx = static_cast<uint16>((e + seg.start) * 2);

               // Determine base pointer to vertex #1
               void* pBase = static_cast<void*>(
                  static_cast<char*>(pBufferStart) +
                     pBuffer->getVertexSize() * baseIdx);

               /* mkultra333 disabling chainTangent calculation, use my AltTangent instead

               // Get index of next item
               size_t nexte = e + 1;
               if (nexte == mMaxElementsPerChain)
                  nexte = 0;

               if (e == seg.head)
               {
                  // No laste, use next item
                  chainTangent = mChainElementList[nexte + seg.start].position - elem.position;
               }
               else if (e == seg.tail)
               {
                  // No nexte, use only last item
                  chainTangent = elem.position - mChainElementList[laste + seg.start].position;
               }
               else
               {
                  // A mid position, use tangent across both prev and next
                  chainTangent = mChainElementList[nexte + seg.start].position - mChainElementList[laste + seg.start].position;

               }

               Vector3 vP1ToEye = eyePos - elem.position;
               Vector3 vPerpendicular = chainTangent.crossProduct(vP1ToEye);
               vPerpendicular.normalise();
               vPerpendicular *= (elem.width * 0.5f);

               Vector3 pos0 = elem.position - vPerpendicular;
               Vector3 pos1 = elem.position + vPerpendicular;
               */

               // mkultra333 alternative tangent
               Vector3 vPerpendicular=AltTangent*elem.width;
               Vector3 pos0 = elem.position - vPerpendicular;
               Vector3 pos1 = elem.position + vPerpendicular;


               float* pFloat = static_cast<float*>(pBase);
               // pos1
               *pFloat++ = pos0.x;
               *pFloat++ = pos0.y;
               *pFloat++ = pos0.z;

               pBase = static_cast<void*>(pFloat);

               if (mUseVertexColour)
               {
                  RGBA* pCol = static_cast<RGBA*>(pBase);
                  Root::getSingleton().convertColourValue(elem.colour, pCol);
                  pCol++;
                  pBase = static_cast<void*>(pCol);
               }

               if (mUseTexCoords)
               {
                  pFloat = static_cast<float*>(pBase);
                  if (mTexCoordDir == TCD_U)
                  {
                     *pFloat++ = elem.texCoord;
                     *pFloat++ = mOtherTexCoordRange[0];
                  }
                  else
                  {
                     *pFloat++ = mOtherTexCoordRange[0];
                     *pFloat++ = elem.texCoord;
                  }
                  pBase = static_cast<void*>(pFloat);
               }

               // pos2
               pFloat = static_cast<float*>(pBase);
               *pFloat++ = pos1.x;
               *pFloat++ = pos1.y;
               *pFloat++ = pos1.z;
               pBase = static_cast<void*>(pFloat);

               if (mUseVertexColour)
               {
                  RGBA* pCol = static_cast<RGBA*>(pBase);
                  Root::getSingleton().convertColourValue(elem.colour, pCol);
                  pCol++;
                  pBase = static_cast<void*>(pCol);
               }

               if (mUseTexCoords)
               {
                  pFloat = static_cast<float*>(pBase);
                  if (mTexCoordDir == TCD_U)
                  {
                     *pFloat++ = elem.texCoord;
                     *pFloat++ = mOtherTexCoordRange[1];
                  }
                  else
                  {
                     *pFloat++ = mOtherTexCoordRange[1];
                     *pFloat++ = elem.texCoord;
                  }
                  pBase = static_cast<void*>(pFloat);
               }

               if (e == seg.tail)
                  break; // last one

               laste = e;

            } // element
         } // segment valid?

      } // each segment

      pBuffer->unlock();
   }

public:
  class CustomOgreBillboardChainFactory : public MovableObjectFactory
  {
  protected:
      MovableObject* createInstanceImpl( const String& name, const NameValuePairList* params)
      {
          printf("CustomOgreBillboardChainFactory::createInstanceImpl()\n");
          size_t maxElements = 20;
          size_t numberOfChains = 1;
          bool useTex = true;
          bool useCol = true;
          bool dynamic = true;
          // optional params
          if (params != 0)
          {
              NameValuePairList::const_iterator ni = params->find("maxElements");
              if (ni != params->end())
              {
                  maxElements = StringConverter::parseUnsignedLong(ni->second);
              }
              ni = params->find("numberOfChains");
              if (ni != params->end())
              {
                  numberOfChains = StringConverter::parseUnsignedLong(ni->second);
              }
              ni = params->find("useTextureCoords");
              if (ni != params->end())
              {
                  useTex = StringConverter::parseBool(ni->second);
              }
              ni = params->find("useVertexColours");
              if (ni != params->end())
              {
                  useCol = StringConverter::parseBool(ni->second);
              }
              ni = params->find("dynamic");
              if (ni != params->end())
              {
                  dynamic = StringConverter::parseBool(ni->second);
              }

          }

          return OGRE_NEW CustomOgreBillboardChain(name, maxElements, numberOfChains, useTex, useCol, dynamic);

      }
  public:
      CustomOgreBillboardChainFactory() {}
      ~CustomOgreBillboardChainFactory() {}

      static String FACTORY_TYPE_NAME;

      const String& getType(void) const { return FACTORY_TYPE_NAME; }
      void destroyInstance( MovableObject* obj )
      {
        OGRE_DELETE  obj;
      }

  };

};

String CustomOgreBillboardChain::CustomOgreBillboardChainFactory::FACTORY_TYPE_NAME = "CustomOgreBillboardChain";

BillboardLine::BillboardLine( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
: Object( scene_manager )
, width_( 0.1f )
, current_line_(0)
, total_elements_(0)
, num_lines_(1)
, max_points_per_line_(100)
, lines_per_chain_(0)
, current_chain_(0)
, elements_in_current_chain_(0)
, custom_billboard_factory_(new CustomOgreBillboardChain::CustomOgreBillboardChainFactory())
{
  if ( !parent_node )
  {
    parent_node = scene_manager_->getRootSceneNode();
  }

  scene_node_ = parent_node->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "BillboardLineMaterial" << count++;
  material_ = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);

  setNumLines(num_lines_);
  setMaxPointsPerLine(max_points_per_line_);
}

BillboardLine::~BillboardLine()
{
  V_Chain::iterator it = chains_.begin();
  V_Chain::iterator end = chains_.end();
  for (;it != end; ++it)
  {
    scene_manager_->destroyBillboardChain(*it);
  }

  scene_manager_->destroySceneNode( scene_node_->getName() );

  Ogre::MaterialManager::getSingleton().remove(material_->getName());
}

Ogre::BillboardChain* BillboardLine::createChain()
{
  std::stringstream ss;
  static int count = 0;
  ss << "BillboardLine chain" << count++;
  Ogre::Root * root = Ogre::Root::getSingletonPtr();
  if (!root->hasMovableObjectFactory(CustomOgreBillboardChain::CustomOgreBillboardChainFactory::FACTORY_TYPE_NAME)) {
    printf("Adding CustomOgreBillboardChain's Factory to OgreRoot\n");
    root->addMovableObjectFactory(custom_billboard_factory_);
  }
  // Ogre::BillboardChain* chain = scene_manager_->createBillboardChain(ss.str());
  Ogre::BillboardChain * chain = static_cast<Ogre::BillboardChain *>(scene_manager_->createMovableObject(
    ss.str(),
    CustomOgreBillboardChain::CustomOgreBillboardChainFactory::FACTORY_TYPE_NAME
  ));
  chain->setMaterialName( material_->getName() );
  scene_node_->attachObject( chain );

  chains_.push_back(chain);

  return chain;
}

void BillboardLine::clear()
{
  V_Chain::iterator it = chains_.begin();
  V_Chain::iterator end = chains_.end();
  for (; it != end; ++it)
  {
    (*it)->clearAllChains();
  }

  current_line_ = 0;
  total_elements_ = 0;
  current_chain_ = 0;
  elements_in_current_chain_ = 0;

  for (V_uint32::iterator it = num_elements_.begin(); it != num_elements_.end(); ++it)
  {
    *it = 0;
  }
}

void BillboardLine::setupChains()
{
  uint32_t total_points = max_points_per_line_ * num_lines_;
  uint32_t num_chains = total_points / MAX_ELEMENTS;
  if (total_points % MAX_ELEMENTS != 0)
  {
    ++num_chains;
  }

  for (uint32_t i = chains_.size(); i < num_chains; ++i)
  {
    createChain();
  }

  lines_per_chain_ = max_points_per_line_ > 0 ? MAX_ELEMENTS / max_points_per_line_ : 1;

  V_Chain::iterator it = chains_.begin();
  V_Chain::iterator end = chains_.end();
  for (;it != end; ++it)
  {
    (*it)->setMaxChainElements(max_points_per_line_);

    // shorten the number of chains in the last bbchain, to avoid memory wasteage
    if (it + 1 == end)
    {
      uint32_t lines_left = num_lines_ % lines_per_chain_;

      // Handle the case where num_lines_ is a multiple of lines_per_chain
      if (lines_left == 0) {
          (*it)->setNumberOfChains(lines_per_chain_);
      }
      else
      {
          (*it)->setNumberOfChains(lines_left);
      }
    }
    else
    {
      (*it)->setNumberOfChains(lines_per_chain_);
    }
  }
}

void BillboardLine::setMaxPointsPerLine(uint32_t max)
{
  max_points_per_line_ = max;

  setupChains();
}

void BillboardLine::setNumLines(uint32_t num)
{
  num_lines_ = num;

  setupChains();

  num_elements_.resize(num);

  for (V_uint32::iterator it = num_elements_.begin(); it != num_elements_.end(); ++it)
  {
    *it = 0;
  }
}

void BillboardLine::newLine()
{
  ++current_line_;

  ROS_ASSERT(current_line_ < num_lines_);
}

void BillboardLine::addPoint( const Ogre::Vector3& point )
{
  addPoint(point, color_);
}

void BillboardLine::addPoint( const Ogre::Vector3& point, const Ogre::ColourValue& color )
{
  ++num_elements_[current_line_];
  ++total_elements_;

  ROS_ASSERT(num_elements_[current_line_] <= max_points_per_line_);

  ++elements_in_current_chain_;
  if (elements_in_current_chain_ > MAX_ELEMENTS)
  {
    ++current_chain_;
    elements_in_current_chain_ = 1;
  }

  Ogre::BillboardChain::Element e;
  e.position = point;
  e.width = width_;
  e.colour = color;
  chains_[current_chain_]->addChainElement(current_line_ % lines_per_chain_ , e);
}

void BillboardLine::setLineWidth( float width )
{
  width_ = width;

  for (uint32_t line = 0; line < num_lines_; ++line)
  {
    uint32_t element_count = num_elements_[line];

    for ( uint32_t i = 0; i < element_count; ++i )
    {
      Ogre::BillboardChain* c = chains_[line / lines_per_chain_];
      Ogre::BillboardChain::Element e = c->getChainElement(line % lines_per_chain_, i);

      e.width = width_;
      c->updateChainElement(line % lines_per_chain_, i, e);
    }
  }
}

void BillboardLine::setPosition( const Ogre::Vector3& position )
{
  scene_node_->setPosition( position );
}

void BillboardLine::setOrientation( const Ogre::Quaternion& orientation )
{
  scene_node_->setOrientation( orientation );
}

void BillboardLine::setScale( const Ogre::Vector3& scale )
{
  // Setting scale doesn't really make sense here
}

void BillboardLine::setColor( float r, float g, float b, float a )
{
  if ( a < 0.9998 )
  {
    material_->getTechnique(0)->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    material_->getTechnique(0)->setDepthWriteEnabled( false );
  }
  else
  {
    material_->getTechnique(0)->setSceneBlending( Ogre::SBT_REPLACE );
    material_->getTechnique(0)->setDepthWriteEnabled( true );
  }

  color_ = Ogre::ColourValue( r, g, b, a );

  for (uint32_t line = 0; line < num_lines_; ++line)
  {
    uint32_t element_count = num_elements_[line];

    for ( uint32_t i = 0; i < element_count; ++i )
    {
      Ogre::BillboardChain* c = chains_[line / lines_per_chain_];
      Ogre::BillboardChain::Element e = c->getChainElement(line % lines_per_chain_, i);

      e.colour = color_;
      c->updateChainElement(line % lines_per_chain_, i, e);
    }
  }
}

const Ogre::Vector3& BillboardLine::getPosition()
{
  return scene_node_->getPosition();
}

const Ogre::Quaternion& BillboardLine::getOrientation()
{
  return scene_node_->getOrientation();
}

} // namespace rviz
