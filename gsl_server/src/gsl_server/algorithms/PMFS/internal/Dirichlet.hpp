#pragma once

#include <cmath>

namespace Dirichlet
{

    // Function to compute the digamma function approximation
    inline double digamma(double x)
    {
        // Coefficients for the Stirling's series approximation of the digamma function
        const double c1 = 1.0 / 12.0;
        const double c2 = -1.0 / 120.0;
        const double c3 = 1.0 / 252.0;
        const double c4 = -1.0 / 240.0;
        const double c5 = 1.0 / 132.0;
        const double c6 = -691.0 / 32760.0;
        const double c7 = 1.0 / 12.0;

        // Adjust for small x (x < 1.0)
        if (x < 1.0)
        {
            return digamma(1.0 + x) - 1.0 / x;
        }

        // Stirling's series approximation for the digamma function
        return std::log(x) - 0.5 / x - c1 / (x * x) + c2 / (x * x * x * x) - c3 / (x * x * x * x * x * x) +
               c4 / (x * x * x * x * x * x * x * x) - c5 / (x * x * x * x * x * x * x * x * x * x) +
               c6 / (x * x * x * x * x * x * x * x * x * x * x) + c7 / (x * x * x * x * x * x * x * x * x * x);
    }

    // Function to compute the expected Shannon entropy
    // can be used with vectors, arrays
    template <typename CollectionFloat>
    inline float expectedShannonEntropy(const CollectionFloat& alpha)
    {
        float A = 0.0;
        float sum_alpha_psi = 0.0;

        // Calculate A and sum_alpha_psi based on the type T
        for (size_t i = 0; i < alpha.size(); ++i)
        {
            if (alpha[i] == 0)
                continue;
            float value = alpha[i];
            A += value;
            sum_alpha_psi += value * digamma(value);
        }

        return digamma(A) - (1.0 / A) * sum_alpha_psi;
    }
} // namespace Dirichlet
