#ifndef MATRIX_H
#define MATRIX_H

//#include <Arduino.h> //Wichtig, da ansonsten keine String-Objekte erzeugt werden können
#include <math.h>

///////////////////////////////////////
// Klassendeklaration UND Definition //
///////////////////////////////////////

//Template Klassen können/sollten nicht in .h und .cpp aufgegliedert werden
//Komplette Klasse muss in die .h Datei

//Klasse für 4x4 Matrizen
template<typename T>
class Matr4x4
{
    public:

    //MEMBERVARIABEN

    //Koeffizienten der Matrix mit Koeffizienten der Einheitsmatrix initialisieren
    T x[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};

    //KONSTRUKTOREN

    //Standardkonstruktor: Einheitsmatrix
    Matr4x4();
    //Matrix mit übergebenen Variablen initialisieren
    Matr4x4(T a, T b, T c, T d, T e, T f, T g, T h, T i, T j, T k, T l, T m, T n, T o, T p);

    //KLASSENMETHODEN

    //Matrix transponieren
    //Rückgabe einer neuen Matrix
    Matr4x4 transpon_neu() const;
    //Matrix transponieren
    //Ursprüngliche Koeffizienten werden überschrieben
    Matr4x4& transpon();
    //Methode zum Multiplizieren einer Matrix mit einem Punkt
    void matr_x_punkt(const Vec3<T> &src, Vec3<T> &dst) const;
    //Methode zum Multiplizieren einer Matrix mit einem Vektor
    void matr_x_vek(const Vec3<T> &src, Vec3<T> &dst) const;
    //Ausgabe der Matrixkoeffizienten als String für den seriellen Monitor
    String printMatr4x4() const;    

    //ÜBERLADENE OPERATOREN

    //Access Operator (Accessor) (Operator [] überladen)
    //Zugriff auf die Koeffizienten der Matrix, ohne die Membervariable zu nennen
    //Bsp: Matr4x4d mat; mat.m[0][3] = x; wird zu: mat[0][3] = x;
    //Jeweils ein Accessor für konstante und nicht konstante Objekte
    const T* operator [] (int i) const;
    T* operator [] (int i);
    //Matrizen multiplizieren (Operator * überladen)
    Matr4x4 operator * (const Matr4x4& x2) const;
};

//KONSTRUKTOREN

//Standardkonstruktor: Einheitsmatrix
template<typename T>
Matr4x4<T>::Matr4x4(){}
//Matrix mit übergebenen Variablen initialisieren
template<typename T>
Matr4x4<T>::Matr4x4(T a, T b, T c, T d, T e, T f, T g, T h, T i, T j, T k, T l, T m, T n, T o, T p)
{
    x[0][0] = a;
    x[0][1] = b;
    x[0][2] = c;
    x[0][3] = d;
    x[1][0] = e;
    x[1][1] = f;
    x[1][2] = g;
    x[1][3] = h;
    x[2][0] = i;
    x[2][1] = j;
    x[2][2] = k;
    x[2][3] = l;
    x[3][0] = m;
    x[3][1] = n;
    x[3][2] = o;
    x[3][3] = p;
}

//KLASSENMETHODEN

//Matrix transponieren
//Rückgabe einer neuen Matrix
template<typename T>
Matr4x4<T> Matr4x4<T>::transpon_neu() const
{
    Matr4x4 x2;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            x2[i][j] = x[j][i];
        }
    }
    return x2;
}
//Matrix transponieren
//Ursprüngliche Koeffizienten werden überschrieben!
template<typename T>
Matr4x4<T>& Matr4x4<T>::transpon()
{
    Matr4x4 x2
   (x[0][0],
    x[1][0],
    x[2][0],
    x[3][0],
    x[0][1],
    x[1][1],
    x[2][1],
    x[3][1],
    x[0][2],
    x[1][2],
    x[2][2],
    x[3][2],
    x[0][3],
    x[1][3],
    x[2][3],
    x[3][3]);
    *this = x2;

    return *this;
}
//Methode zum Multiplizieren einer Matrix mit einem Punkt
//Obwohl Punkte und Vektoren durch die gleiche Klasse repräsentiert werden,
//gibt es mathematisch Unterschiede: Translatation macht nur bei Punkten und nicht bei Vektoren Sinn
//Darüber hinaus sollen Punkte als homogene Koordinaten dargestellt werden (=4. Variable h)
//h ist in aller Regel = 1, ausser die Matrix ist eine Projektionsmatrix (für perspektivische Darstellung)
//Dann müssen die x,y,z-Koordinaten durch h geteilt werden
template<typename T>
void Matr4x4<T>::matr_x_punkt(const Vec3<T> &src, Vec3<T> &dst) const
{
    T a, b, c, h;

    //ROW-MAJOR Variante
    //Koeffizienten für Verschiebung stehen in der vierten Zeile
    /*
    a = x[0][0] * src[0] + x[1][0] * src[1] + x[2][0] * src[2] + x[3][0];
    b = x[0][1] * src[0] + x[1][1] * src[1] + x[2][1] * src[2] + x[3][1];
    c = x[0][2] * src[0] + x[1][2] * src[1] + x[2][2] * src[2] + x[3][2];
    h = x[0][3] * src[0] + x[1][3] * src[1] + x[2][3] * src[2] + x[3][3];
    */
    //COLUMN-MAJOR Variante
    //Koeffizienten für Verschiebung stehen in der vierten Spalte
    a = x[0][0] * src[0] + x[0][1] * src[1] + x[0][2] * src[2] + x[0][3];
    b = x[1][0] * src[0] + x[1][1] * src[1] + x[1][2] * src[2] + x[1][3];
    c = x[2][0] * src[0] + x[2][1] * src[1] + x[2][2] * src[2] + x[2][3];
    h = x[3][0] * src[0] + x[3][1] * src[1] + x[3][2] * src[2] + x[3][3];

    if(h != 1 && h != 0)
    {
        dst.x = a / h;
        dst.y = b / h;
        dst.z = c / h;
    }
    //Wenn h gleich 0 oder 1 ist, wird x,y,z direkt zurückgegeben
    else
    {
        dst.x = a;
        dst.y = b;
        dst.z = c;
    }
}
//Methode zum Multiplizieren einer Matrix mit einem Vektor
//Matrixkoeffizienten für Verschiebung werden ignoriert
//h wird nicht berechnet
template<typename T>
void Matr4x4<T>::matr_x_vek(const Vec3<T> &src, Vec3<T> &dst) const
{
    T a, b, c;

    //ROW-MAJOR Variante
    /*
    a = x[0][0] * src[0] + x[1][0] * src[1] + x[2][0] * src[2];
    b = x[0][1] * src[0] + x[1][1] * src[1] + x[2][1] * src[2];
    c = x[0][2] * src[0] + x[1][2] * src[1] + x[2][2] * src[2];
    */
    //COLUMN-MAJOR Variante
    a = x[0][0] * src[0] + x[0][1] * src[1] + x[0][2] * src[2];
    b = x[1][0] * src[0] + x[1][1] * src[1] + x[1][2] * src[2];
    c = x[2][0] * src[0] + x[2][1] * src[1] + x[2][2] * src[2];

    dst.x = a;
    dst.y = b;
    dst.z = c;
}
//Ausgabe der Matrixkoeffizienten als String für den seriellen Monitor
template<typename T>
String Matr4x4<T>::printMatr4x4() const
{
  String strg_ausgabe = String("");
  String strg1 = String("|");
  String strg2 = String("\t");
  String strg3 = String("\n");
  
  for(int i=0; i<4; i++)
  {
    strg_ausgabe += strg1;
    strg_ausgabe += x[i][0];
    strg_ausgabe += strg2;
    strg_ausgabe += x[i][1];
    strg_ausgabe += strg2;
    strg_ausgabe += x[i][2];
    strg_ausgabe += strg2;
    strg_ausgabe += x[i][3];
    strg_ausgabe += strg2;
    strg_ausgabe += strg1;
    strg_ausgabe += strg3;
  }
  
  return strg_ausgabe;
}

//ÜBERLADENE OPERATOREN

//Access Operator (Accessor) (Operator [] überladen)
//Zugriff auf die Koeffizienten der Matrix, ohne die Membervariable zu nennen
//Bsp: Matr4x4d mat; mat.m[0][3] = x; wird zu: mat[0][3] = x;
//Jeweils ein Accessor für konstante und nicht konstante Objekte
template<typename T>
const T* Matr4x4<T>::operator [] (int i) const
{
    return x[i];
}
template<typename T>
T* Matr4x4<T>::operator [] (int i)
{
    return x[i];
}
//Matrizen multiplizieren (Operator * überladen)
template<typename T>
Matr4x4<T> Matr4x4<T>::operator * (const Matr4x4& x2) const
{
    Matr4x4 x3;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            x3[i][j] =
            x[i][0] * x2[0][j] +
            x[i][1] * x2[1][j] +
            x[i][2] * x2[2][j] +
            x[i][3] * x2[3][j];
        }
    }
    return x3;
}

//Neuen Datentyp definieren mit Klasse Matr4x4 als double
typedef Matr4x4<double> Matr4x4d;

#endif

