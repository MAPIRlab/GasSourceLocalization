#include "KernelDMVW.hpp"
#include "gsl_server/algorithms/Common/Utils/Math.hpp"

namespace GSL::KernelDMVW
{
    GasMap::GasMap(Grid2D<Occupancy> occupancyMap, const Params& parameters)
        : cells(occupancyMap.data.size()),
          grid(cells, occupancyMap),
          params(parameters)
    {
    }

    void GasMap::AddReading(float concentration, Vector2 wind, Vector2 measurePosition)
    {
        Vector2Int measuredIndices = grid.metadata.coordinatesToIndices(measurePosition);
        if (!grid.metadata.indicesInBounds(measuredIndices) || !grid.occupancyAt(measuredIndices))
            return;

        Vector2Int maxUpdateIndices = grid.metadata.coordinatesToIndices(measurePosition + Vector2{params.kernelSigma, params.kernelSigma});
        Vector2Int minUpdateIndices = grid.metadata.coordinatesToIndices(measurePosition - Vector2{params.kernelSigma, params.kernelSigma});
        maxUpdateIndices.x = std::clamp(maxUpdateIndices.x, 0, grid.metadata.dimensions.x - 1);
        maxUpdateIndices.y = std::clamp(maxUpdateIndices.y, 0, grid.metadata.dimensions.y - 1);
        minUpdateIndices.x = std::clamp(minUpdateIndices.x, 0, grid.metadata.dimensions.x - 1);
        minUpdateIndices.y = std::clamp(minUpdateIndices.y, 0, grid.metadata.dimensions.y - 1);
        AABB2DInt aabb(minUpdateIndices, maxUpdateIndices);

        // Important note:
        // instead of calculating the full covariance matrix for the oriented gaussian, this implementation just stretches it along the x and y axes
        // the angle is then passed into evaluate2DGaussian() as a separate parameter. This makes the computation a bit simpler
        float windAngle = std::atan2(wind.y, wind.x);
        float windSpeed = wind.norm();
        for (Vector2Int indices : aabb)
        {
            Vector2 pos = grid.metadata.indicesToCoordinates(indices);
            Vector2 offset = pos - measurePosition;

            Vector2 finalSigma(
                params.kernelSigma / (1 + params.kernelStretchConstant * windSpeed / params.kernelSigma), // semi-minor ellipse axis
                params.kernelSigma + params.kernelStretchConstant * windSpeed                             // semi-major axis
            );

            float weight = Utils::evaluate2DGaussian(offset, finalSigma, windAngle);
            KernelCell& cell = grid.dataAt(indices);
            cell.omega += weight;
            cell.confidence = 1 - std::exp(-cell.omega / params.sigmaOmega);
            cell.meanAndVariance.Update(concentration, weight);
        }
    }

} // namespace GSL::KernelDMVW