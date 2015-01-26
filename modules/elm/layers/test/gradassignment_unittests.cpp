#include "elm/layers/gradassignment.h"

#include <gtest/gtest.h>

#include <boost/graph/adjacency_list.hpp>

#include <opencv2/core.hpp>

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




    const int A=3;  ///< no. of nodes in G
    const int I=3;  ///< no. of nodes in g

    // adjacency matrices
    Mat1f g_ab(A, A);
    randn(g_ab, 0.f, 10.f);
    g_ab = abs(g_ab);

    Mat1f g_ij(I, I);
    randn(g_ij, 0.f, 10.f);
    g_ij = abs(g_ij);
    g_ij = g_ab;

    std::cout<<"adjacency matrices"<<std::endl;
    std::cout<<g_ab<<std::endl;

    cv::Size2i sz(I, A);


    const float epislon=1e-10;

    float beta0 = 0.5f;
    const float BETA_MAX = 10.f;
    bool is_m_converged = false;
    bool is_m_hat_converged = false;
    int nb_iterations_0 = 0;
    int nb_iterations_1 = 0;
    const int MAX_ITERATIONS_0 = 4;  ///< max. no. of iterations allowed at each value of beta
    const int MAX_ITERATIONS_1 = 30; ///< max. no. of iterations allowed for Sinkhorn's method
    const float BETA_RATE = 1.075f;

    Mat1f m_ai_hat(sz.height+1, sz.width+1, 0.f);    ///< match matrix variables including the slacks
    for(int i=0; i<sz.height; i++) {
        m_ai_hat(i, i) = 1.f;
    }
    m_ai_hat.row(sz.height).setTo(epislon);
    m_ai_hat.col(sz.width).setTo(epislon);
    //m_ai.copyTo(m_ai_hat(cv::Rect2i(0, 0, m_ai.rows, m_ai.cols)));

    Mat1f m_ai = m_ai_hat(Rect2i(0, 0, sz.width, sz.height));        ///< match matrix variables, excluding the slacks

    float beta = beta0;
    while(beta < BETA_MAX) { // A

        is_m_converged = false;
        nb_iterations_0 = 0;
        std::cout<<"beta="<<beta<<std::endl;
        while(!is_m_converged && nb_iterations_0 < MAX_ITERATIONS_0) { // B

            std::cout<<"B"<<nb_iterations_0<<std::endl;
            Mat1f q_ai;      ///< partial derivative of Ewg with respect to M_ai

            // c_aibj
            Mat1f c_aibj;
            Mat1f i_sigma_g_ab_over_b;
            reduce(g_ab, i_sigma_g_ab_over_b, 0, cv::REDUCE_SUM);
            i_sigma_g_ab_over_b *= I;

            Mat1f a_sigma_g_ij_over_j;
            reduce(g_ij, a_sigma_g_ij_over_j, 1, cv::REDUCE_SUM);
            a_sigma_g_ij_over_j *= A;

            c_aibj = a_sigma_g_ij_over_j*i_sigma_g_ab_over_b;
            std::cout<<"c_aibj="<<std::endl<<c_aibj<<std::endl;
            // c_aibj

            // sum_bj_Mbj
            float sum_bj_Mbj = cv::sum(m_ai)[0];
            //reduce(m_ai, sum_bj_Mbj, 1, cv::REDUCE_SUM);

            // sum_bj_Mbj
            // q_ai
            cv::multiply(c_aibj, sum_bj_Mbj, q_ai);

            std::cout<<"h1"<<std::endl;
            // q_ai
            // softassign
            Mat1f m_ai0;
            cv::exp(beta*q_ai, m_ai0);

            Mat1f m_ai_hat0 = m_ai_hat.clone();    ///< match matrix variables including the slacks

            is_m_hat_converged = false;
            nb_iterations_1 = 0;
            while(!is_m_hat_converged && nb_iterations_1 < MAX_ITERATIONS_1) { // C

                std::cout<<"C"<<nb_iterations_1<<std::endl;
                // update m hat by normalizing across all rows
                Mat1f row_sums_i;
                reduce(m_ai_hat0, row_sums_i, 1, cv::REDUCE_SUM);
                row_sums_i = cv::repeat(row_sums_i, 1, m_ai_hat0.cols);
                Mat1f m_ai_hat1 = m_ai_hat0/row_sums_i;

                // update m hat by normalizing across all columns
                Mat1f col_sums_i;
                reduce(m_ai_hat1, col_sums_i, 0, cv::REDUCE_SUM);
                col_sums_i = cv::repeat(col_sums_i, m_ai_hat0.rows, 1);
                m_ai_hat0 = m_ai_hat1/col_sums_i;

                nb_iterations_1++;

                std::cout<<"m_ai_hat"<<std::endl;
                std::cout<<m_ai_hat<<std::endl;
                std::cout<<"m_ai_hat0"<<std::endl;
                std::cout<<m_ai_hat0<<std::endl;
                is_m_hat_converged = cv::sum(abs(m_ai_hat-m_ai_hat0))[0] < epislon;

            } // end C

            nb_iterations_0++;

            std::cout<<"m_ai"<<std::endl;
            std::cout<<m_ai<<std::endl;
            std::cout<<"m_ai0"<<std::endl;
            std::cout<<m_ai0<<std::endl;
            is_m_converged = cv::sum(abs(m_ai-m_ai0))[0] < epislon;

        } // end B

        beta *= BETA_RATE;
    } // end A

    std::cout<<"m_ai"<<std::endl;
    std::cout<<m_ai<<std::endl;
    std::cout<<"m_ai_hat"<<std::endl;
    std::cout<<m_ai_hat<<std::endl;
}
