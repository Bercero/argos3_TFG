#include "kilobot_custom_communication_medium.h"
#include "kilobot_custom_entity.h"
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/space/positional_indices/grid.h>
#include <argos3/core/utility/configuration/argos_exception.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/kilobot_custom/simulator/kilobot_custom_measures.h>

namespace argos {

   /****************************************/
   /****************************************/

   CKilobotCustomCommunicationMedium::CKilobotCustomCommunicationMedium() :
      m_pcKilobotCustomIndex(NULL),
      m_pcGridUpdateOperation(NULL),
      m_pcRNG(NULL),
      m_fRxProb(0.0),
      m_bIgnoreConflicts(false)
   {
   }

   /****************************************/
   /****************************************/

   CKilobotCustomCommunicationMedium::~CKilobotCustomCommunicationMedium() {
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationMedium::Init(TConfigurationNode& t_tree) {
      try {
         CMedium::Init(t_tree);
         /* Get the arena center and size */
         CVector3 cArenaCenter;
         CVector3 cArenaSize;
         TConfigurationNode& tArena = GetNode(CSimulator::GetInstance().GetConfigurationRoot(), "arena");
         GetNodeAttribute(tArena, "size", cArenaSize);
         GetNodeAttributeOrDefault(tArena, "center", cArenaCenter, cArenaCenter);
         /* Create the positional index for embodied entities */
         UInt32 unXCells = Ceil(cArenaSize.GetX() / (KILOBOT_CUSTOM_RADIUS+KILOBOT_CUSTOM_RADIUS));
         UInt32 unYCells = Ceil(cArenaSize.GetY() / (KILOBOT_CUSTOM_RADIUS+KILOBOT_CUSTOM_RADIUS));
         CGrid<CKilobotCustomCommunicationEntity>* pcGrid = new CGrid<CKilobotCustomCommunicationEntity>(
            cArenaCenter - cArenaSize * 0.5f, cArenaCenter + cArenaSize * 0.5f,
            unXCells, unYCells, 1);
         m_pcGridUpdateOperation = new CKilobotCustomCommunicationEntityGridEntityUpdater(*pcGrid);
         pcGrid->SetUpdateEntityOperation(m_pcGridUpdateOperation);
         m_pcKilobotCustomIndex = pcGrid;
         /* Set probability of receiving a message */
         GetNodeAttributeOrDefault(t_tree, "message_drop_prob", m_fRxProb, m_fRxProb);
         m_fRxProb = 1.0 - m_fRxProb;
         /* Create random number generator */
         m_pcRNG = CRandom::CreateRNG("argos");
         /* Whether or not to ignore conflicts due to channel congestion */
         GetNodeAttributeOrDefault(t_tree, "ignore_conflicts", m_bIgnoreConflicts, m_bIgnoreConflicts);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error in initialization of the range-and-bearing medium", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationMedium::PostSpaceInit() {
      Update();
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationMedium::Reset() {
      /* Reset positional index of KilobotCustom entities */
      m_pcKilobotCustomIndex->Reset();
      /* Delete adjacency matrix */
      for(TAdjacencyMatrix::iterator it = m_tCommMatrix.begin();
          it != m_tCommMatrix.end();
          ++it) {
         it->second.clear();
      }
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationMedium::Destroy() {
      delete m_pcKilobotCustomIndex;
      if(m_pcGridUpdateOperation != NULL)
         delete m_pcGridUpdateOperation;
   }

   /****************************************/
   /****************************************/

   static size_t HashKilobotCustomPair(const std::pair<CKilobotCustomCommunicationEntity*, CKilobotCustomCommunicationEntity*>& c_pair) {
      return
         reinterpret_cast<size_t>(c_pair.first) ^
         reinterpret_cast<size_t>(c_pair.second);
   }

   void CKilobotCustomCommunicationMedium::Update() {
      /*
       * Update positional index of KilobotCustom entities
       */
      m_pcKilobotCustomIndex->Update();
      /*
       * Delete obsolete adjacency matrices
       */
      for(TAdjacencyMatrix::iterator it = m_tCommMatrix.begin();
          it != m_tCommMatrix.end();
          ++it) {
         it->second.clear();
      }
      m_tTxNeighbors.clear();
      /*
       * Construct the adjacency matrix of transmitting robots
       */
      /* Buffer for the communicating entities */
      CSet<CKilobotCustomCommunicationEntity*,SEntityComparator> cOtherKilobotCustoms;
      /* This map contains the pairs that have already been checked */
      unordered_map<size_t, std::pair<CKilobotCustomCommunicationEntity*, CKilobotCustomCommunicationEntity*> > mapPairsAlreadyChecked;
      /* Iterator for the above structure */
      unordered_map<size_t, std::pair<CKilobotCustomCommunicationEntity*, CKilobotCustomCommunicationEntity*> >::iterator itPair;
      /* Used as test key */
      std::pair<CKilobotCustomCommunicationEntity*, CKilobotCustomCommunicationEntity*> cTestKey;
      /* Used as hash for the test key */
      size_t unTestHash;
      /* The square distance between two KilobotCustoms */
      Real fSqDistance;
      /* Go through the KilobotCustom entities */
      for(TAdjacencyMatrix::iterator it = m_tCommMatrix.begin();
          it != m_tCommMatrix.end();
          ++it) {
         /* Get a reference to the current KilobotCustom entity */
         CKilobotCustomCommunicationEntity& cKilobotCustom = *reinterpret_cast<CKilobotCustomCommunicationEntity*>(GetSpace().GetEntityVector()[it->first]);
         /* Is this robot trying to transmit? */
         if(cKilobotCustom.GetTxStatus() == CKilobotCustomCommunicationEntity::TX_ATTEMPT) {
            /* Yes, add it to the list of transmitting robots */
            m_tTxNeighbors[cKilobotCustom.GetIndex()];
            /* Get the list of KilobotCustoms in range */
            cOtherKilobotCustoms.clear();
            m_pcKilobotCustomIndex->GetEntitiesAt(cOtherKilobotCustoms, cKilobotCustom.GetPosition());
            /* Go through the KilobotCustoms in range */
            for(CSet<CKilobotCustomCommunicationEntity*,SEntityComparator>::iterator it2 = cOtherKilobotCustoms.begin();
                it2 != cOtherKilobotCustoms.end();
                ++it2) {
               /* Get a reference to the neighboring KilobotCustom */
               CKilobotCustomCommunicationEntity& cOtherKilobotCustom = **it2;
               /* First, make sure the entities are not the same and
                  that they are both transmitting */
               if(&cKilobotCustom != &cOtherKilobotCustom &&
                  cOtherKilobotCustom.GetTxStatus() == CKilobotCustomCommunicationEntity::TX_ATTEMPT) {
                  /* Proceed if the pair has not been checked already */
                  if(&cKilobotCustom < &cOtherKilobotCustom) {
                     cTestKey.first = &cKilobotCustom;
                     cTestKey.second = &cOtherKilobotCustom;
                  }
                  else {
                     cTestKey.first = &cOtherKilobotCustom;
                     cTestKey.second = &cKilobotCustom;
                  }
                  unTestHash = HashKilobotCustomPair(cTestKey);
                  itPair = mapPairsAlreadyChecked.find(unTestHash);
                  if(itPair == mapPairsAlreadyChecked.end() ||   /* Pair does not exist */
                     itPair->second.first != cTestKey.first ||   /* Pair exists, but first KilobotCustom involved is different */
                     itPair->second.second != cTestKey.second) { /* Pair exists, but second KilobotCustom involved is different */
                     /* Mark this pair as already checked */
                     mapPairsAlreadyChecked[unTestHash] = cTestKey;
                     /* Calculate square distance */
                     fSqDistance = SquareDistance(cKilobotCustom.GetPosition(),
                                                  cOtherKilobotCustom.GetPosition());
                     if(fSqDistance < Square(cOtherKilobotCustom.GetTxRange())) {
                        /* cKilobotCustom receives cOtherKilobotCustom's message */
                        m_tTxNeighbors[cKilobotCustom.GetIndex()].insert(&cOtherKilobotCustom);
                     }
                     if(fSqDistance < Square(cKilobotCustom.GetTxRange())) {
                        /* cOtherKilobotCustom receives cKilobotCustom's message */
                        m_tTxNeighbors[cOtherKilobotCustom.GetIndex()].insert(&cKilobotCustom);
                     }
                  } /* pair check */
               } /* entity identity + transmit check */
            } /* neighbors loop */
         } /* transmission check */
      } /* robot loop */
      /*
       * Go through transmitting robots and broadcast messages
       */
      /* Buffer to store the intersection data */
      SEmbodiedEntityIntersectionItem sIntersectionItem;
      /* Loop over transmitting robots */
      for(TAdjacencyMatrix::iterator it = m_tTxNeighbors.begin();
          it != m_tTxNeighbors.end();
          ++it) {
         /* Is this robot conflicting? */
         if(m_bIgnoreConflicts ||
            it->second.empty() ||
            m_pcRNG->Uniform(CRange<UInt32>(0, it->second.size() + 1)) == 0) {
            /* The robot can transmit */
            /* Get a reference to the current KilobotCustom entity */
            CKilobotCustomCommunicationEntity& cKilobotCustom = *reinterpret_cast<CKilobotCustomCommunicationEntity*>(GetSpace().GetEntityVector()[it->first]);
            /* Change its transmission status */
            cKilobotCustom.SetTxStatus(CKilobotCustomCommunicationEntity::TX_SUCCESS);
            /* Go through its neighbors */
            cOtherKilobotCustoms.clear();
            m_pcKilobotCustomIndex->GetEntitiesAt(cOtherKilobotCustoms, cKilobotCustom.GetPosition());
            for(CSet<CKilobotCustomCommunicationEntity*,SEntityComparator>::iterator it2 = cOtherKilobotCustoms.begin();
                it2 != cOtherKilobotCustoms.end();
                ++it2) {
               /* Get a reference to the neighboring KilobotCustom entity */
               CKilobotCustomCommunicationEntity& cOtherKilobotCustom = **it2;
               /* Make sure the robots are different */
               if(&cKilobotCustom != &cOtherKilobotCustom) {
                  /* Calculate distance */
                  fSqDistance = SquareDistance(cKilobotCustom.GetPosition(),
                                               cOtherKilobotCustom.GetPosition());
                  /* If robots are within transmission range and transmission succeeds... */
                  if(fSqDistance < Square(cKilobotCustom.GetTxRange()) &&
                     m_pcRNG->Bernoulli(m_fRxProb)) {
                     /* cOtherKilobotCustom receives cKilobotCustom's message */
                     m_tCommMatrix[cOtherKilobotCustom.GetIndex()].insert(&cKilobotCustom);
                  }
               } /* identity check */
            } /* neighbor loop */
         } /* conflict check */
      } /* transmitters loop */
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationMedium::AddEntity(CKilobotCustomCommunicationEntity& c_entity) {
      m_tCommMatrix.insert(
         std::make_pair<ssize_t, CSet<CKilobotCustomCommunicationEntity*,SEntityComparator> >(
            c_entity.GetIndex(), CSet<CKilobotCustomCommunicationEntity*,SEntityComparator>()));
      m_pcKilobotCustomIndex->AddEntity(c_entity);
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationMedium::RemoveEntity(CKilobotCustomCommunicationEntity& c_entity) {
      m_pcKilobotCustomIndex->RemoveEntity(c_entity);
      TAdjacencyMatrix::iterator it = m_tCommMatrix.find(c_entity.GetIndex());
      if(it != m_tCommMatrix.end())
         m_tCommMatrix.erase(it);
   }

   /****************************************/
   /****************************************/

   const CSet<CKilobotCustomCommunicationEntity*,SEntityComparator>& CKilobotCustomCommunicationMedium::GetKilobotCustomsCommunicatingWith(CKilobotCustomCommunicationEntity& c_entity) const {
      TAdjacencyMatrix::const_iterator it = m_tCommMatrix.find(c_entity.GetIndex());
      if(it != m_tCommMatrix.end()) {
         return it->second;
      }
      else {
         THROW_ARGOSEXCEPTION("KilobotCustom entity \"" << c_entity.GetId() << "\" is not managed by the KilobotCustom medium \"" << GetId() << "\"");
      }
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationMedium::SendOHCMessageTo(CKilobotCustomEntity& c_robot,
                                                      message_t* pt_message) {
      /*
       * old   | new   | action
       * ------+-------+---------------------
       * ~null | ~null | delete old, copy new
       * ~null |  null | delete old, set null
       *  null | ~null | copy new
       *  null |  null | nothing to do
       */
      /* Get old message and delete it if necessary */
      message_t* ptOldMsg = GetOHCMessageFor(c_robot);
      if(ptOldMsg != NULL) delete ptOldMsg;
      /* Set message */
      m_mapOHCMessages[c_robot.GetIndex()] = NULL;
      if(pt_message != NULL)
         m_mapOHCMessages[c_robot.GetIndex()] = new message_t(*pt_message);
   }

   /****************************************/
   /****************************************/

   void CKilobotCustomCommunicationMedium::SendOHCMessageTo(std::vector<CKilobotCustomEntity*>& vec_robots,
                                                      message_t* pt_message) {
      for(size_t i = 0; i < vec_robots.size(); ++i) {
         /*
          * old   | new   | action
          * ------+-------+---------------------
          * ~null | ~null | delete old, copy new
          * ~null |  null | delete old, set null
          *  null | ~null | copy new
          *  null |  null | nothing to do
          */
         /* Get old message and delete it if necessary */
         message_t* ptOldMsg = GetOHCMessageFor(*vec_robots[i]);
         if(ptOldMsg != NULL) delete ptOldMsg;
         /* Set message */
         m_mapOHCMessages[vec_robots[i]->GetIndex()] = NULL;
         if(pt_message != NULL)
            m_mapOHCMessages[vec_robots[i]->GetIndex()] = new message_t(*pt_message);
      }
   }

   /****************************************/
   /****************************************/

   message_t* CKilobotCustomCommunicationMedium::GetOHCMessageFor(CKilobotCustomEntity& c_robot) {
      /* Look for robot in map */
      unordered_map<ssize_t, message_t*>::iterator it = m_mapOHCMessages.find(c_robot.GetIndex());
      /* Return entry if robot found, NULL otherwise */
      return (it == m_mapOHCMessages.end()) ? NULL : (it->second);
   }

   /****************************************/
   /****************************************/

   REGISTER_MEDIUM(CKilobotCustomCommunicationMedium,
                   "kilobot_custom_communication",
                   "Carlo Pinciroli [ilpincy@gmail.com]",
                   "1.0",
                   "It simulates communication across KilobotCustom robots.",
                   "This medium is required to simulate communication across KilobotCustoms. It works as\n"
                   "follows:\n"
                   "1. The medium calculates which robots can transmit at each time step. Every\n"
                   "   robot is assigned a non-transmission period. At the end of this period, the\n"
                   "   robot attempts transmission. It is successful with a probability that is\n"
                   "   inversely proportional to the number of transmitting robots in range. If\n"
                   "   successful, robot waits until the next period to transmit again. Otherwise\n"
                   "   it tries at the next time step.\n"
                   "2. It broadcasts the messages of the robots that can transmit. It is possible\n"
                   "   to specify the probability of message dropping by a robot as an optional\n"
                   "   parameter (see below).\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "<kilobot_custom_communication id=\"kbc\" />\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "It is possible to specify the probability with which a robot a drops an incoming\n"
                   "message. This is done by setting the attribute \"message_drop_prob\". When set\n"
                   "to 0, no message is ever dropped; when set to 1, every message is dropped.\n\n"
                   "<kilobot_custom_communication id=\"kbc\" message_drop_prob=\"0.25\" />\n\n"
                   "It is also possible to ignore the effect of channel congestion. When two robots\n"
                   "are trying to send a message at the same time, a message conflict occurs. The\n"
                   "default behavior is to allow robots to complete message delivery according to a\n"
                   "random choice. If you don't want conflicts to be simulated, set the flag\n"
                   "'ignore_conflicts' to 'true':\n\n"
                   "<kilobot_custom_communication id=\"kbc\" ignore_conflicts=\"true\" />\n"
                   ,
                   "Under development"
      );

   /****************************************/
   /****************************************/

}
