#ifndef __Xi2_h__
#define __Xi2_h__

#include <vector>

#include "mt.h"

namespace st {
    class Xi2Calculator {
    public:
        Xi2Calculator(std::vector<mt::UInt>& data1, std::vector<mt::UInt>& data2):
        m_xi2Value(0.0)
        {
            if (data1.size() != data2.size()) {
                glob::assert(false);
                return;
            }

            //get total amount of elements in first distribution
            mt::UInt n_elems_1 = _getTotalElems(data1);
            mt::UInt n_elems_2 = _getTotalElems(data2);

            mt::UInt n_groups = static_cast<mt::UInt>(data1.size());
            for (mt::UInt group_idx = 0; group_idx < n_groups; group_idx++) {
                mt::UInt n_group_elems_1 = data1[group_idx];
                mt::UInt n_group_elems_2 = data2[group_idx];

                if (n_group_elems_1 + n_group_elems_2 == 0) {
                    continue;
                }

                mt::Real one_div_gr1_gr2 = 1.0 / (n_group_elems_1 + n_group_elems_2);
                mt::Real gr1_div_n1 = static_cast<mt::Real>(n_group_elems_1) / n_elems_1;
                mt::Real gr1_div_n2 = static_cast<mt::Real>(n_group_elems_2) / n_elems_2;
                mt::Real g1_d_n1_g2_d_n2_2 = (gr1_div_n1 - gr1_div_n2) * (gr1_div_n1 - gr1_div_n2);
                m_xi2Value += one_div_gr1_gr2 * g1_d_n1_g2_d_n2_2;
            }
            m_xi2Value *= n_elems_1 * n_elems_2;
        }

        mt::Real getValue(void) { return m_xi2Value; }
    private:
        mt::UInt _getTotalElems(std::vector<mt::UInt>& data)
        {
            mt::UInt n_total_elems = 0;
            mt::UInt data_size = static_cast<mt::UInt>(data.size());
            for (mt::UInt data_iter = 0; data_iter < data_size; data_iter++) {
                n_total_elems += data[data_iter];
            }
            return n_total_elems;
        }

        mt::Real m_xi2Value;
    };

} // namespace st

#endif //__Xi2_h__