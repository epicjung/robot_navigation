#include <string>
#include <cmath>

using namespace std;

class Gaussian_Mixture_Model{
private:

public:
	string type_covariance;
	int dimension_data;
	int number_gaussian_components;

	double *weight;

	double **mean;

	double **diagonal_covariance;
	double ***covariance;

	Gaussian_Mixture_Model();
	Gaussian_Mixture_Model(string type_covariance, int dimension_data, int number_gaussian_components);
	~Gaussian_Mixture_Model();

	void SetParameter(string type_covariance, int dimension_data, int number_gaussian_components);

	void Initialize(int number_data, double **data);

	void Load_Parameter(string path);
	void Save_Parameter(string path);

	int Classify(double data[]);

	double Calculate_Likelihood(double data[]);
	double Calculate_Likelihood(double data[], double gaussian_distribution[]);
	double Expectaion_Maximization(int number_data, double **data);
	double Gaussian_Distribution(double data[], int component_index);
	double Gaussian_Distribution(double data[], double mean[], double diagonal_covariance[]);
	double Gaussian_Distribution(double data[], double mean[], double **covariance);
};
