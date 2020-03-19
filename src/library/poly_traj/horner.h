#ifndef HORNER_H_
#define HORNER_H_

#include <stdio.h>
#include <stdlib.h>
#include <vector>

//using namespace std;

// HORNERSCHEMA: Berechnnung aller Ableitungen eines Polynoms an der Stelle x
// Eingabe: Grad n; Zeiger auf Koeffizienten a, Argument x
// Ausgabe: Wert des Polynoms als Rueckgabewert
// Zeiger auf w, w[i] enthält den Wert der iten Ableitung
//ACHTUNG: Die Zeiger a und w müssen auf Felder der Länge n+1 zeigen.

//double Horner(int n, double* a, double x, double* w) {
//	for (int i=0; i<n+1; i++) w[i]=a[n]; // w[0] wird Rueckgabewert
//    for (int i=n-1; i>=0; i--) {
//        w[0]=w[0]*x + a[i];
//        for (int j=1; j<=i; j++) w[j] = w[j]*x + w[j-1];
//        }
//        for (int i=2; i<=n; i++){
//            for (int j=2; j<=i; j++) w[i]=w[i]*j; // Multiplikation mit i!
//        }
//        return w[0];
//}


// New implementation:
int horner(const std::vector<double>& a, double x, std::vector<double>& w);
#endif // HORNER_H_
