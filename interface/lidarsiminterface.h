#pragma once
#include <memory>
#include <vector>
#include "lidarsource.h"
#include "raycastresult.h"

//TODO - Defined elsewhere.
//Model will be an OSG-side object should contain an ID and info whether it is static. The model should represent a whole concept (i.e. vehicle, not a vehicle wheel)
//Models should be relatively low-poly for performance
class Model;
typedef std::string ModelID; //TODO

/// <summary>
/// Interface class for the Optix lidar module
/// Responsible for: raycasting for given models and lidar sources
/// Not responsible for: moving models or lidars around, managing raycast frequencies, computing vector directions of lidar rays.
/// </summary>
class LidarSimInterface
{
    public:
        /// <summary>
        /// A factory method which is the only way to construct the object.
        /// The caller is responsible for ownership/lifetime.
        /// </summary>
        virtual std::unique_ptr<LidarSimInterface> CreateLidarSim() = 0;

        /// <summary>
        /// Add model. A local structure will be created for the model. The model is identified with its ID
        /// </summary>
        virtual void addModel(const Model &model) = 0;

        /// <summary>
        /// Update model(s). Models are matched by IDs. Models added earlier which are not in the vector are unchanged.
        /// Models which are in the vector but weren't added before are ignored (TODO - warning should be signalled?)
        /// </summary>
        virtual void updateModels(const std::vector<Model> &models) = 0;

        /// <summary>
        /// Remove model by IDs
        /// </summary>
        virtual void removeModel(const ModelID &id) = 0;

        /// <summary>
        /// Perform raycasting for a single lidar source (a single sensor)
        /// </summary>
        virtual RaycastResult raycast(const LidarSource &source) = 0;

        /// <summary>
        /// Batch raycast. Useful when we don't care about the particular source and lidars have synchronized frequency
        /// </summary>
        virtual RaycastResults raycastBatch(const std::vector<LidarSource> &sources) = 0;

        /// <summary>
        /// An empty virtual destructor is also provided for polymorphism.
        /// </summary>
        virtual ~LidarSimInterface() {};

    protected:
        LidarSimInterface() {};

    private:
        LidarSimInterface(const LidarSimInterface &) = delete;
        LidarSimInterface& operator=(LidarSimInterface const&) = delete;
}
