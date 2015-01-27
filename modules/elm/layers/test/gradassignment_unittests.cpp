#include "elm/layers/gradassignment.h"

#include <gtest/gtest.h>

#include <boost/graph/adjacency_list.hpp>

#include <opencv2/core.hpp>

#include "elm/core/cv/mat_utils_inl.h"

using namespace std;
using namespace boost;
using namespace cv;
using namespace elm;
namespace bg=boost::graph;

class GradAssignmentTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {

    }


};

int tril_flat(const Mat1f &src, Mat1f &dst)
{
    int nb_ab = 0;
    for(int i=src.rows; i>0; i--) {
        nb_ab += i;
    }

    dst = Mat1f(1, nb_ab);

    int i=0;
    for(int r=0; r<src.rows; r++) {
        for(int c=0; c<=r; c++) {

            dst(i++) = src(r, c);
        }
    }

    return nb_ab;
}

int tril_sparse(const Mat1f &src, std::vector<Mat1f > &dst)
{
    dst.clear();
    dst.reserve(src.rows);

    int nb_ab = 0;
    for(int i=src.rows-1; i>0; i--) {
        nb_ab += i;
    }

    for(int r=1; r<src.rows; r++) {

        Mat1f m(1, r);
        for(int c=0; c<r; c++) {

            m(c) = src(r, c);
        }
        dst.push_back(m);
    }

    return nb_ab;
}

float compatibility_func(float w1, float w2)
{
    float c;
    if(w1 == 0.f || w2 == 0.f) {

        c = 0.f;
    }
    else {
        c = 1-3.f*abs(w1-w2);
    }
    return c;
}

cv::Mat1f rand_shuffle_rows(cv::Mat1f &src)
{
    Mat1i new_order = ARange_<int>(0, src.cols, 1);
    random_shuffle(new_order.begin(), new_order.end());

    Mat1f dst(src.size());
    for(int i=0; i<src.cols; i++) {

        src.row(i).copyTo(dst.row(new_order(i)));
    }

    return dst;
}

TEST_F(GradAssignmentTest, Test)
{
//    // Property types
//    typedef property<edge_weight_t, int> EdgeWeightProperty;
//    typedef property<vertex_name_t, std::string,
//      property<vertex_index2_t, int> > VertexProperties;

//    // Graph type
//    typedef adjacency_list<vecS, vecS, undirectedS,
//      VertexProperties, EdgeWeightProperty> Graph;

//    // Graph instance
//    Graph g;

//    // Property accessors
//    property_map<Graph, vertex_name_t>::type
//      city_name = get(vertex_name, g);
//    property_map<Graph, vertex_index2_t>::type
//      city_index2 = get(vertex_index2, g);
//    property_map<Graph, edge_weight_t>::type
//      edge_distance = get(edge_weight, g);

//    // Create the vertices
//    typedef graph_traits<Graph>::vertex_descriptor Vertex;
//    Vertex u1;
//    u1 = add_vertex(g);
//    city_name[u1] = "Los Angeles";
//    city_index2[u1] = 3;

//      Vertex u2;
//      u2 = add_vertex(g);
//      city_name[u2] = "Bakersfield";
//      city_index2[u2] = 2;

//    Vertex u3;
//    u3 = add_vertex(g);
//    city_name[u3] = "New York";
//    city_index2[u3] = 1;

//    // Create the edges
//    typedef graph_traits<Graph>::edge_descriptor Edge;
//    Edge e1;
//    e1 = (add_edge(u1, u2, g)).first;
//    edge_distance[e1] = 100;

//    Edge e2;
//    e2 = add_edge(u1, u3, g).first;
//    edge_distance[e2] = 2500;

//    // Iterate through the vertices and print them out
//    typedef graph_traits<Graph>::vertex_iterator vertex_iter;
//    std::pair<vertex_iter, vertex_iter> vp;
//    for (vp = vertices(g); vp.first != vp.second; ++vp.first)
//      std::cout << city_name[*vp.first] << " " << city_index2[*vp.first] << std::endl;
//    std::cout << std::endl;

//    // Iterate through the edges and print them out
//    Vertex v1, v2;
//    typedef graph_traits<Graph>::edge_iterator edge_iter;
//    std::pair<edge_iter, edge_iter> ep;
//    edge_iter ei, ei_end;
//    for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei)
//      std::cout << edge_distance[*ei] << endl;

    const int A=4;  ///< no. of nodes in G
    const int I=A;  ///< no. of nodes in g

    // generate random adjacency matrices
    Mat1f g_ab(A, A);
    Mat1i tmp(A, A);
    int s = 1000;
    randu(tmp, 0, s);
    g_ab = tmp/static_cast<float>(s);

    // distance from node to itself is 0
    for(int r=0; r<g_ab.rows; r++) {

        g_ab(r, r) = 0.f;
    }
    // make symmetrical
    for(int r=0; r<g_ab.rows; r++) {

        for(int c=0; c<g_ab.rows; c++) {

            g_ab(r, c) = g_ab(c, r);
        }
    }

    Mat1f g_ij(I, I);
    g_ij = g_ab.clone();    // make them equal
    Mat1f noise(g_ij.size());
    randn(noise, 0.f, 0.5f);
    g_ij += noise;
    g_ij.setTo(0.f, g_ij < 0.f);
    g_ij.setTo(1.f, g_ij > 1.f);

    for(int r=0; r<g_ij.rows; r++) {

        g_ij(r, r) = 0.f;
        for(int c=0; c<g_ij.cols; c++) {

            g_ij(r, c) = g_ij(c, r);
        }
    }

    std::cout<<"adjacency matrices"<<std::endl;
    std::cout<<"g_ab="<<std::endl;
    std::cout<<g_ab<<std::endl;
    std::cout<<"g_ij="<<std::endl;
    std::cout<<g_ij<<std::endl;

    // compatibility matrix c_aibj

//    Mat1f g_ab_tril_flat;
//    int nb_ab = tril_flat(g_ab, g_ab_tril_flat);

//    Mat1f g_ij_tril_flat;
//    int nb_ij = tril_flat(g_ij, g_ij_tril_flat);


//    std::cout<<"g_ab_trill_flat"<<std::endl;
//    std::cout<<g_ab_tril_flat<<std::endl;
//    std::cout<<"g_ij_trill_flat"<<std::endl;
//    std::cout<<g_ij_tril_flat<<std::endl;

//    Mat1f c_aibj(nb_ab, nb_ij, 0.f);
//    for(int a=0; a<nb_ab; a++) {

//        float g_ab_tmp = g_ab_tril_flat(a);
//        if(g_ab_tmp != 0.f) {

//            for(int i=0; i<nb_ij; i++) {

//                float g_ij_tmp = g_ij_tril_flat(i);
//                if(g_ij_tmp != 0.f) {

//                    c_aibj(a, i) = compatibility_func(g_ab_tmp, g_ij_tmp);
//                }
//            }
//        }
//    }
    Mat1f c_ai(A, I, 0.f);

    std::vector<Mat1f > g_ab_tril_sparse;
    tril_sparse(g_ab, g_ab_tril_sparse);

    std::vector<Mat1f > g_ij_tril_sparse;
    tril_sparse(g_ij, g_ij_tril_sparse);


    for(size_t i=0; i<g_ab_tril_sparse.size(); i++) {

        cout<<g_ab_tril_sparse[i]<<std::endl;
    }

    for(size_t i=0; i<g_ij_tril_sparse.size(); i++) {

        cout<<g_ij_tril_sparse[i]<<std::endl;
    }

    for(int a=0; a<A; a++) {

        Mat1f g_ab_row = g_ab.row(a);
        for(int i=0; i<I; i++) {

            Mat1f g_ij_row = g_ij.row(i);

            float sigma_c_aibj_over_bj = 0.f;
            for(size_t b=0; b<g_ab_row.total(); b++) {

                for(size_t j=0; j<g_ij_row.total(); j++) {

                    sigma_c_aibj_over_bj += compatibility_func(g_ab_row(b), g_ij_row(j));
                }
            }

            c_ai(a, i) = sigma_c_aibj_over_bj;
        }
    }

    std::cout<<"c_ai"<<std::endl;
    std::cout<<c_ai<<std::endl;

    //cv::Size2i sz(I, A);

    const float epislon=1e-2;

    float beta0 = 0.5f;
    const float BETA_MAX = 10.f;
    bool is_m_converged = false;
    //bool is_m_hat_converged = false;
    int nb_iterations_0 = 0;
    int nb_iterations_1 = 0;
    const int MAX_ITERATIONS_0 = 4;  ///< max. no. of iterations allowed at each value of beta
    const int MAX_ITERATIONS_1 = 30; ///< max. no. of iterations allowed for Sinkhorn's method
    const float BETA_RATE = 1.075f;

    // hypothesize on matching matrix
    Mat1f m_ai(A, I, 1.f+epislon);
//    for(int i=0; i<m_ai.rows; i++) {
//        m_ai(i, i) = 1.f;
//    }
//    // permute?

//    Mat1f m_ai_hat(sz.height+1, sz.width+1);    ///< match matrix variables including the slacks
//    m_ai.copyTo(m_ai_hat(Rect2i(0, 0, sz.width, sz.height)));
//    m_ai_hat.row(sz.height).setTo(epislon);
//    m_ai_hat.col(sz.width).setTo(epislon);

//    // share data between match matrices
//    m_ai = m_ai_hat(Rect2i(0, 0, sz.width, sz.height));        ///< match matrix variables, excluding the slacks

    float beta = beta0;
    while(beta < BETA_MAX) { // A

        //is_m_converged = false;
        nb_iterations_0 = 0;
        //std::cout<<"beta="<<beta<<std::endl;
        while(!is_m_converged && nb_iterations_0 < MAX_ITERATIONS_0) { // B

            //std::cout<<"B"<<nb_iterations_0<<std::endl;
            Mat1f q_ai;      ///< partial derivative of Ewg with respect to M_ai

            // c_aibj
//            Mat1f c_aibj;
//            Mat1f i_sigma_g_ab_over_b;
//            reduce(g_ab, i_sigma_g_ab_over_b, 0, cv::REDUCE_SUM);
//            i_sigma_g_ab_over_b *= I;

//            Mat1f a_sigma_g_ij_over_j;
//            reduce(g_ij, a_sigma_g_ij_over_j, 1, cv::REDUCE_SUM);
//            a_sigma_g_ij_over_j *= A;

//            c_aibj = a_sigma_g_ij_over_j*i_sigma_g_ab_over_b;
//            std::cout<<"c_aibj="<<std::endl<<c_aibj<<std::endl;
            // c_aibj

            // sum_bj_Mbj
            float sum_bj_Mbj = cv::sum(m_ai)[0];
            //reduce(m_ai, sum_bj_Mbj, 1, cv::REDUCE_SUM);

            // sum_bj_Mbj
            // q_ai
            //cv::multiply(c_aibj, sum_bj_Mbj, q_ai);
            cv::multiply(c_ai, sum_bj_Mbj, q_ai);

            //std::cout<<"h1"<<std::endl;
            // q_ai
            // softassign
            Mat1f m_ai0;
            cv::exp(beta*q_ai, m_ai0);

            //Mat1f m_ai_hat0 = m_ai_hat.clone();    ///< match matrix variables including the slacks

            //is_m_hat_converged = false;
            nb_iterations_1 = 0;
            //while(!is_m_hat_converged && nb_iterations_1 < MAX_ITERATIONS_1) { // C
            while(!is_m_converged && nb_iterations_1 < MAX_ITERATIONS_1) { // C

                //std::cout<<"C"<<nb_iterations_1<<std::endl;

                // update m by normalizing across all rows
                Mat1f row_sums_i;
                reduce(m_ai0, row_sums_i, 1, cv::REDUCE_SUM);
                row_sums_i = cv::repeat(row_sums_i, 1, m_ai0.cols);
                Mat1f m_ai1 = m_ai0/row_sums_i;

                // update m by normalizing across all columns
                Mat1f col_sums_i;
                reduce(m_ai1, col_sums_i, 0, cv::REDUCE_SUM);
                col_sums_i = cv::repeat(col_sums_i, m_ai0.rows, 1);
                m_ai0 = m_ai1/col_sums_i;

                nb_iterations_1++;

//                std::cout<<"m_ai"<<std::endl;
//                std::cout<<m_ai<<std::endl;
//                std::cout<<"m_ai0"<<std::endl;
//                std::cout<<m_ai0<<std::endl;
                is_m_converged = cv::sum(abs(m_ai-m_ai0))[0] < epislon;

                m_ai = m_ai0;

            } // end C

            nb_iterations_0++;

//            std::cout<<"m_ai"<<std::endl;
//            std::cout<<m_ai<<std::endl;
//            std::cout<<"m_ai0"<<std::endl;
//            std::cout<<m_ai0<<std::endl;
            //is_m_converged = cv::sum(abs(m_ai-m_ai0))[0] < epislon;

        } // end B

        beta *= BETA_RATE;
    } // end A

    std::cout<<"m_ai"<<std::endl;
    std::cout<<m_ai<<std::endl;
//    std::cout<<"m_ai_hat"<<std::endl;
//    std::cout<<m_ai_hat<<std::endl;
}
