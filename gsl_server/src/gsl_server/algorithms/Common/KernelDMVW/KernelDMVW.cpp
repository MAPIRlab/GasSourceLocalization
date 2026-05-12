#include "KernelDMVW.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"

namespace GSL::KernelDMVW
{
    GasMap::GasMap(Grid2D<Occupancy> occupancyMap, const Params& parameters)
        : cells(occupancyMap.data.size()),
          metadata(occupancyMap.metadata),
          occupancy(occupancyMap.occupancy),
          params(parameters)
    {
    }

    void GasMap::AddReading(float concentration, Vector2 wind, Vector2 measurePosition)
    {
        Grid2D<KernelCell> grid(cells, occupancy, metadata);
        Vector2Int measuredIndices = grid.metadata.coordinatesToIndices(measurePosition);
        if (!grid.metadata.indicesInBounds(measuredIndices) || !grid.occupancyAt(measuredIndices))
            return;

        float windAngle = -std::atan2(wind.y, wind.x) + M_PI * 0.5; // I'm honestly not sure why we need to rotate this
        float windSpeed = wind.norm();
        // Important note:
        // instead of calculating the full covariance matrix for the oriented gaussian, this implementation just stretches it along the x and y axes
        // the angle is then passed into evaluate2DGaussian() as a separate parameter. This makes the computation a bit simpler
        Vector2 finalSigma(
            params.kernelSigma / (1 + params.kernelStretchConstant * windSpeed / params.kernelSigma), // semi-minor ellipse axis
            params.kernelSigma + params.kernelStretchConstant * windSpeed                             // semi-major axis
        );

        // calculate the update bounds
        Vector2 p1 = vmath::rotate(Vector2{finalSigma.x, 0}, windAngle);
        Vector2 p2 = vmath::rotate(Vector2{0, finalSigma.y}, windAngle);
        Vector2 updateBounds{};
        updateBounds.x = std::max({std::abs(p1.x), std::abs(p2.x)});
        updateBounds.y = std::max({std::abs(p1.y), std::abs(p2.y)});
        updateBounds = updateBounds * 3;

        Vector2Int maxUpdateIndices = grid.metadata.coordinatesToIndices(measurePosition + updateBounds);
        Vector2Int minUpdateIndices = grid.metadata.coordinatesToIndices(measurePosition - updateBounds);
        maxUpdateIndices.x = std::clamp(maxUpdateIndices.x, 0, grid.metadata.dimensions.x - 1);
        maxUpdateIndices.y = std::clamp(maxUpdateIndices.y, 0, grid.metadata.dimensions.y - 1);
        minUpdateIndices.x = std::clamp(minUpdateIndices.x, 0, grid.metadata.dimensions.x - 1);
        minUpdateIndices.y = std::clamp(minUpdateIndices.y, 0, grid.metadata.dimensions.y - 1);
        AABB2DInt aabb(minUpdateIndices, maxUpdateIndices);

        for (Vector2Int indices : aabb)
        {
            Vector2 pos = grid.metadata.indicesToCoordinates(indices);
            Vector2 offset = pos - measurePosition;

            float weight = Utils::evaluate2DGaussian(offset, finalSigma, windAngle);
            KernelCell& cell = grid.dataAt(indices);
            cell.omega += std::pow(weight, params.omegaConcentrationSpatial);
            cell.confidence = 1 - std::exp(-cell.omega / params.sigmaOmega);
            cell.meanAndVariance.Update(concentration, weight);
        }
    }

} // namespace GSL::KernelDMVW