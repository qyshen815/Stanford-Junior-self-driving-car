#include "horner.h"

//int horner(const std::vector<double>& a, double x, std::vector<double>& w) {
//    int n = a.size()-1;
//    if ( (int)w.size() != n+1) {return -1;}
//
//	double an = a[n];
//	w.assign(n+1, an); // w[0] wird Rueckgabewert
//	std::vector<double>::const_reverse_iterator it=a.rbegin(); it++;
//	for (int i=n-1; it!=a.rend(); it++, i--) {
//		std::vector<double>::iterator it2 = w.begin();
//		std::vector<double>::iterator it3 = it2; it3++;
//		*(it2) = *(it2)*x + (*it);
//		for (int j=1; j<=i; j++, it2++, it3++) {
//			*(it3) = *(it3)*x + (*it2);
//	    }
//	}
//
//	std::vector<double>::iterator it2 = w.begin(); it2++; it2++;
//	for (int i=2; i<=n; i++, it2++){
//		for (int j=2; j<=i; j++) (*it2)*=j; // Multiplikation mit i!
//		}
//	return 0;
//}



int horner(const std::vector<double>& a, double x, std::vector<double>& w) {
    int n = a.size()-1;
    if ( (int)w.size() != n+1) {return -1;}

    w.assign(n+1, a[n]); // w[0] wird Rueckgabewert
//	for (int i=0; i<n+1; i++) {w[i]=a[n];} // w[0] wird Rueckgabewert
	for (int i=n-1; i>=0; i--) {
		w[0]=w[0]*x + a[i];
		for (int j=1; j<=i; j++) w[j] = w[j]*x + w[j-1];
	}
	for (int i=2; i<=n; i++){
		for (int j=2; j<=i; j++) w[i]=w[i]*j; // Multiplikation mit i!
		}
	return 0;
}
