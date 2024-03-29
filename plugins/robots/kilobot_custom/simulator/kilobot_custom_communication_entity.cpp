/**
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "kilobot_custom_communication_entity.h"
#include "kilobot_custom_communication_medium.h"
#include <argos3/core/utility/string_utilities.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CKilobotCustomCommunicationEntity::CKilobotCustomCommunicationEntity(CComposableEntity* pc_parent,
                                                            const std::string& str_id,
                                                            size_t un_msg_size,
                                                            Real f_range,
                                                            SAnchor& s_anchor,
                                                            CEmbodiedEntity& c_entity_body) :
      CPositionalEntity(pc_parent,
                        str_id),
      m_psAnchor(&s_anchor),
      m_fTxRange(f_range),
      m_pcEntityBody(&c_entity_body),
      m_eTxStatus(TX_NONE),
      m_pcMedium(NULL) {
      Disable();
      SetInitPosition(s_anchor.Position);
      SetPosition(GetInitPosition());
      SetInitOrientation(s_anchor.Orientation);
      SetOrientation(GetInitOrientation());
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationEntity::Reset() {
      m_eTxStatus = TX_NONE;
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationEntity::Update() {
      if(m_eTxStatus == TX_SUCCESS) m_eTxStatus = TX_NONE;
      SetPosition(m_psAnchor->Position);
      SetOrientation(m_psAnchor->Orientation);
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationEntity::SetEnabled(bool b_enabled) {
      /* Perform generic enable behavior */
      CEntity::SetEnabled(b_enabled);
      /* Perform specific enable behavior */
      if(b_enabled) {
         /* Enable body anchor */
         if(m_psAnchor)
            m_psAnchor->Enable();
         /* Enable entity in medium */
         if(m_pcMedium && GetIndex() >= 0)
            m_pcMedium->AddEntity(*this);
      }
      else {
         /* Disable body anchor */
         if(m_psAnchor)
            m_psAnchor->Disable();
         /* Disable entity in medium */
         if(m_pcMedium)
            m_pcMedium->RemoveEntity(*this);
      }
   }

   /****************************************/
   /****************************************/

   bool CKilobotCustomCommunicationEntity::HasMedium() const {
      return m_pcMedium != NULL;
   }

   /****************************************/
   /****************************************/

   CKilobotCustomCommunicationMedium& CKilobotCustomCommunicationEntity::GetMedium() {
      return *m_pcMedium;
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationEntity::SetMedium(CKilobotCustomCommunicationMedium& c_medium) {
      m_pcMedium = &c_medium;
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationEntitySpaceHashUpdater::operator()(CAbstractSpaceHash<CKilobotCustomCommunicationEntity>& c_space_hash,
                                                       CKilobotCustomCommunicationEntity& c_element) {
      /* Calculate the position of the center of the kilobot_custom communication entity in the space hash */
      c_space_hash.SpaceToHashTable(m_nCenterI,
                                    m_nCenterJ,
                                    m_nCenterK,
                                    c_element.GetPosition());
      /* Update the cells in a sphere around it */
      SInt32 nRangeI = c_space_hash.SpaceToHashTable(c_element.GetTxRange(), 0);
      SInt32 nRangeJ;
      SInt32 nRangeK;
      for(SInt32 i = 0; i <= nRangeI; ++i) {
         nRangeJ =
            c_space_hash.SpaceToHashTable(
               ::sqrt(
                  Square(c_element.GetTxRange()) -
                  Square(c_space_hash.HashTableToSpace(i, 0))
                  ),
               1);
         for(SInt32 j = 0; j <= nRangeJ; ++j) {
            nRangeK =
               c_space_hash.SpaceToHashTable(
                  ::sqrt(
                     Square(c_element.GetTxRange()) -
                     Square(c_space_hash.HashTableToSpace(j, 1))
                     ),
                  2);
            for(SInt32 k = 0; k <= nRangeK; ++k) {
               if(i > 0) {
                  /*
                   * i > 0
                   */
                  if(j > 0) {
                     /*
                      * i > 0
                      * j > 0
                      */
                     if(k > 0) {
                        /*
                         * i > 0
                         * j > 0
                         * k > 0
                         */
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ + j, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ + j, m_nCenterK - k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ - j, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ - j, m_nCenterK - k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ + j, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ + j, m_nCenterK - k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ - j, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ - j, m_nCenterK - k, c_element);
                     }
                     else {
                        /*
                         * i > 0
                         * j > 0
                         * k == 0
                         */
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ + j, m_nCenterK, c_element);
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ - j, m_nCenterK, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ + j, m_nCenterK, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ - j, m_nCenterK, c_element);
                     }
                  }
                  else {
                     /*
                      * i > 0
                      * j == 0
                      */
                     if(k > 0) {
                        /*
                         * i > 0
                         * j == 0
                         * k > 0
                         */
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ, m_nCenterK - k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ, m_nCenterK - k, c_element);
                     }
                     else {
                        /*
                         * i > 0
                         * j == 0
                         * k == 0
                         */
                        c_space_hash.UpdateCell(m_nCenterI + i, m_nCenterJ, m_nCenterK, c_element);
                        c_space_hash.UpdateCell(m_nCenterI - i, m_nCenterJ, m_nCenterK, c_element);
                     }
                  }
               }
               else {
                  /*
                   * i == 0
                   */
                  if(j > 0) {
                     /*
                      * i == 0
                      * j > 0
                      */
                     if(k > 0) {
                        /*
                         * i == 0
                         * j > 0
                         * k > 0
                         */
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ + j, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ + j, m_nCenterK - k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ - j, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ - j, m_nCenterK - k, c_element);
                     }
                     else {
                        /*
                         * i == 0
                         * j > 0
                         * k == 0
                         */
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ + j, m_nCenterK, c_element);
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ - j, m_nCenterK, c_element);
                     }
                  }
                  else {                     
                     /*
                      * i == 0
                      * j == 0
                      */
                     if(k > 0) {
                        /*
                         * i == 0
                         * j == 0
                         * k > 0
                         */
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ, m_nCenterK + k, c_element);
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ, m_nCenterK - k, c_element);
                     }
                     else {
                        /*
                         * i == 0
                         * j == 0
                         * k == 0
                         */
                        c_space_hash.UpdateCell(m_nCenterI, m_nCenterJ, m_nCenterK, c_element);
                     }
                  }
               }
            }
         }
      }
   }

   /****************************************/
   /****************************************/

   class CSpaceOperationAddCKilobotCustomCommunicationEntity : public CSpaceOperationAddEntity {
   public:
      void ApplyTo(CSpace& c_space, CKilobotCustomCommunicationEntity& c_entity) {
         /* Add entity to space - this ensures that the RAB entity
          * gets an id before being added to the RAB medium */
         c_space.AddEntity(c_entity);
         /* Enable the RAB entity, if it's enabled - this ensures that
          * the entity gets added to the RAB if it's enabled */
         c_entity.SetEnabled(c_entity.IsEnabled());
      }
   };

   class CSpaceOperationRemoveCKilobotCustomCommunicationEntity : public CSpaceOperationRemoveEntity {
   public:
      void ApplyTo(CSpace& c_space, CKilobotCustomCommunicationEntity& c_entity) {
         /* Disable the entity - this ensures that the entity is
          * removed from the RAB medium */
         c_entity.Disable();
         /* Remove the RAB entity from space */
         c_space.RemoveEntity(c_entity);
      }
   };

   REGISTER_SPACE_OPERATION(CSpaceOperationAddEntity, CSpaceOperationAddCKilobotCustomCommunicationEntity, CKilobotCustomCommunicationEntity);
   REGISTER_SPACE_OPERATION(CSpaceOperationRemoveEntity, CSpaceOperationRemoveCKilobotCustomCommunicationEntity, CKilobotCustomCommunicationEntity);

   /****************************************/
   /****************************************/

   CKilobotCustomCommunicationEntityGridCellUpdater::CKilobotCustomCommunicationEntityGridCellUpdater(CGrid<CKilobotCustomCommunicationEntity>& c_grid) :
      m_cGrid(c_grid) {}
   
   bool CKilobotCustomCommunicationEntityGridCellUpdater::operator()(SInt32 n_i,
                                                      SInt32 n_j,
                                                      SInt32 n_k,
                                                      CGrid<CKilobotCustomCommunicationEntity>::SCell& s_cell) {
      /* Update cell */
      m_cGrid.UpdateCell(n_i, n_j, n_k, *m_pcEntity);
      /* Continue with other cells */
      return true;
   }
   
   void CKilobotCustomCommunicationEntityGridCellUpdater::SetEntity(CKilobotCustomCommunicationEntity& c_entity) {
      m_pcEntity = &c_entity;
   }

   CKilobotCustomCommunicationEntityGridEntityUpdater::CKilobotCustomCommunicationEntityGridEntityUpdater(CGrid<CKilobotCustomCommunicationEntity>& c_grid) :
      m_cGrid(c_grid),
      m_cCellUpdater(c_grid) {}

   bool CKilobotCustomCommunicationEntityGridEntityUpdater::operator()(CKilobotCustomCommunicationEntity& c_entity) {
      try {
         m_cCellUpdater.SetEntity(c_entity);
         m_cGrid.ForCellsInBoxRange(c_entity.GetPosition(),
                                    CVector3(c_entity.GetTxRange(),
                                             c_entity.GetTxRange(),
                                             c_entity.GetTxRange()),
                                    m_cCellUpdater);
         /* Continue with the other entities */
         return true;
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("While updating the kilobot_custom entity grid for kilobot_custom entity \"" << c_entity.GetContext() << c_entity.GetId() << "\"", ex);
      }
   }

   /****************************************/
   /****************************************/

}
