#ifndef QP_WRAPPER_INL_H_
#define QP_WRAPPER_INL_H_

public:
//\name Accessors and mutators
//\{
inline QPMatrix &hessian_mat() 			{return hessian_mat_;}
inline QPVector &gradient_vec() 			{return gradient_vec_;};

inline QPMatrix &constr_mat() 			{return cstr_mat_;}
inline QPVector &uc_bounds_vec() 		{return constr_u_bounds_vec_;};
inline QPVector &lc_bounds_vec() 		{return constr_l_bounds_vec_;};
inline QPVector &uv_bounds_vec() 		{return var_u_bounds_vec_;};
inline QPVector &lv_bounds_vec() 		{return var_l_bounds_vec_;};

inline void num_var (int nbvar) 			{num_variables_ = nbvar;}
inline int num_var () const 				{return num_variables_;}

inline void num_var_max (int nbvar) 		{num_variables_max_ = nbvar;}
inline int num_var_max () const 			{return num_variables_max_;}

inline void num_constr(int num_constr) 		{num_constr_ = num_constr;}
inline int num_constr() const 				{return num_constr_;}

inline void num_constr_max(int num_constr) 	{num_constr_max_ = num_constr;}
inline int num_constr_max() const 			{return num_constr_max_;}
//\}


#endif /* QP_WRAPPER_INL_H_ */
