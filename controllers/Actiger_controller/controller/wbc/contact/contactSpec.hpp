#ifndef CONTACT_SPEC
#define CONTACT_SPEC

#include "cppTypes.h"

namespace quadruped
{
    
    class ContactSpec {
    public:
        ContactSpec(size_t dim) : dim_contact(dim), b_set_contact(false) {
            idx_Fz = dim - 1;  
            fr_des = DVec<double>::Zero(dim);
        }
        virtual ~ContactSpec() {}

        size_t GetDim() const { return dim_contact; }
        size_t GetDimRFConstraint() const { return cstrMtx.rows(); }
        size_t GetFzIndex() const { return idx_Fz; }

        DMat<double> GetContactJacobian() { return Jc; }
        DVec<double> GetJcDotQdot() { return JcDotQdot; }
        void UnsetContact() { b_set_contact = false; }

        DMat<double> GetRFConstraintMtx() { return cstrMtx; }
        DMat<double> GetRFConstraintVecL() { return ieq_vec_l; }
        DMat<double> GetRFConstraintVecU() { return ieq_vec_u; }
        DVec<double> GetRFDesired() { return fr_des; }
        void SetRFDesired(DVec<double> p_fr_des) { fr_des = p_fr_des; }

        bool UpdateContactSpec() {
            UpdateJc();
            UpdateJcDotQdot();
            UpdateUf();
            UpdateInequalityVector();
            b_set_contact = true;
            return true;
        }

    protected:
        virtual bool UpdateJc() = 0;
        virtual bool UpdateJcDotQdot() = 0;
        virtual bool UpdateUf() = 0;
        virtual bool UpdateInequalityVector() = 0;

        int idx_Fz;
        DMat<double> cstrMtx;
        DVec<double> fr_des;
        DVec<double> ieq_vec_l;
        DVec<double> ieq_vec_u;

        Mat3x18<double> Jc;
        DVec<double> JcDotQdot;
        size_t dim_contact;
        bool b_set_contact;
    };




}

#endif
