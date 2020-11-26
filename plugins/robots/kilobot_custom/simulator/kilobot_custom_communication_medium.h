#ifndef KILOBOT_CUSTOM_COMMUNICATION_MEDIUM_H
#define KILOBOT_CUSTOM_COMMUNICATION_MEDIUM_H

namespace argos {
   class CKilobotCustomCommunicationMedium;
   class CKilobotCustomCommunicationEntity;
   class CKilobotCustomEntity;
}

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/medium/medium.h>
#include <argos3/core/simulator/space/positional_indices/positional_index.h>
#include <argos3/plugins/robots/kilobot_custom/simulator/kilobot_custom_communication_entity.h>

namespace argos {

   class CKilobotCustomCommunicationMedium : public CMedium {

   public:

      /** Defines the adjacency matrix */
      typedef unordered_map<ssize_t, CSet<CKilobotCustomCommunicationEntity*, SEntityComparator> > TAdjacencyMatrix;

   public:

      /**
       * Class constructor.
       */
      CKilobotCustomCommunicationMedium();

      /**
       * Class destructor.
       */
      virtual ~CKilobotCustomCommunicationMedium();

      virtual void Init(TConfigurationNode& t_tree);
      virtual void PostSpaceInit();
      virtual void Reset();
      virtual void Destroy();
      virtual void Update();

      /**
       * Adds the specified entity to the list of managed entities.
       * @param c_entity The entity to add.
       */
      void AddEntity(CKilobotCustomCommunicationEntity& c_entity);
      
      /**
       * Removes the specified entity from the list of managed entities.
       * @param c_entity The entity to remove.
       */
      void RemoveEntity(CKilobotCustomCommunicationEntity& c_entity);

      /**
       * Returns an immutable vector of entities that can communicate with the given entity.
       * @param c_entity The wanted entity.
       * @return An immutable vector of entities that can communicate with the given entity.       
       * @throws CARGoSException If the passed entity is not managed by this medium.
       */
      const CSet<CKilobotCustomCommunicationEntity*,SEntityComparator>& GetKilobotCustomsCommunicatingWith(CKilobotCustomCommunicationEntity& c_entity) const;

      /**
       * Returns a reference to the adjacency matrix.
       * @return A reference to the adjacency matrix.
       */
      TAdjacencyMatrix& GetCommMatrix(){
          return m_tCommMatrix;
      }

      /**
       * Sends a message to the given robot, as if it were done by the overhead controller.
       * Only one message per time step can be set.
       * Once set, a message stays until explicitly erased.
       * To erase a message, set it to NULL.
       * @param c_robot The message recipient.
       * @param pt_message The message payload.
       */
      void SendOHCMessageTo(CKilobotCustomEntity& c_robot,
                            message_t* pt_message);

      /**
       * Sends a message to the given robots, as if it were done by the overhead controller.
       * Only one message per time step can be set.
       * Once set, a message stays until explicitly erased.
       * To erase a message, set it to NULL.
       * @param vec_robots The message recipients.
       * @param pt_message The message payload.
       */
      void SendOHCMessageTo(std::vector<CKilobotCustomEntity*>& vec_robots,
                            message_t* Message);

      /**
       * Returns the OHC message for the given KilobotCustom.
       * @returns the OHC message payload (or NULL if no message is associated to the given robot)
       */
      message_t* GetOHCMessageFor(CKilobotCustomEntity& c_robot);

   private:

      /** The adjacency matrix, that associates each entity with the entities that communicate with it */
      TAdjacencyMatrix m_tCommMatrix;

      /** The adjacency matrix of neighbors of a transmitting robot who are also transmitting */
      TAdjacencyMatrix m_tTxNeighbors;

      /** A positional index for the kilobot_custom communication entities */
      CPositionalIndex<CKilobotCustomCommunicationEntity>* m_pcKilobotCustomIndex;

      /** The update operation for the grid positional index */
      CKilobotCustomCommunicationEntityGridEntityUpdater* m_pcGridUpdateOperation;

      /** A list of messages set through SendOHCMessageTo() */
      unordered_map<ssize_t, message_t*> m_mapOHCMessages;

      /** Random number generator */
      CRandom::CRNG* m_pcRNG;

      /** Probability of receiving a message */
      Real m_fRxProb;

      /** Whether to ignore communication conflicts due to channel congestion */
      bool m_bIgnoreConflicts;

   };

}

#endif
