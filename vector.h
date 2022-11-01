#ifndef VECTOR_H
#define VECTOR_H

#include <Arduino.h> //Wichtig, da ansonsten keine String-Objekte erzeugt werden können
#include <math.h>

///////////////////////////////////////
// Klassendeklaration UND Definition //
///////////////////////////////////////

//Template Klassen können/sollten nicht in .h und .cpp aufgegliedert werden
//Komplette Klasse muss in die .h Datei

//Klasse für Vektoren, Punke und Normalen im dreidimensionalen Raum
//Klasse als Template angelegt, um die Vektordaten in Verschiedenen Typen speichern zu können
//also z.B. int, double, usw.
template<typename T>
class Vec3
{
    public:

    //MEMBERVARIABEN

    //Variablen für x,y,z-Werte
    T x, y, z;

    //KONSTRUKTOREN

    //Standardkonstruktor: Vektorobjekt ohne Parameter erzeugen
    //x, y und z werden als 0 initialisiert
    Vec3();
    //Vektorobjekt mit nur einem Parameter erzeugen
    //Übergebener Wert wird dann zu x, y und z
    Vec3(const T &xx);
    //Vektorobjekt mit übergebenen Parametern erzeugen
    Vec3(T xx, T yy, T zz);

    //KLASSENMETHODEN

    //Betrag des Vektors bzw des Ortsvektors des Punktes berechnen
    T betrag() const;
    //Skalarprodukt mit einem übergebenen Vektor berechnen
    //0: Vektoren orthogonal, 1: Vektoren parallel
    T skalarprod(const Vec3<T> &v) const;
    //Vektorprodukt (= Kreuzprodukt) mit einem übergebenen Vektor berechnen
    //Ergibt einen Vektor, der auf jedem der beiden Vektoren senkrecht (orthogonal) steht
    Vec3<T> vektorprod(const Vec3<T> &v) const;
    //Vektor normalisieren
    //Rückgabe des Normalenvektors vom Typ Vec3
    Vec3<T> normalisieren_neu() const;
    //Vektor normalisieren
    //Vektor wird bei dieser Variante überschrieben
    Vec3<T>& normalisieren();
    //Ausgabe der Vektorkoeffizienten als String für den seriellen Monitor
    String printVec3() const;

    //ÜBERLADENE OPERATOREN

    //Access Operator (Accessor) (Operator [] überladen)
    //Auf die Koordinaten des Vektors kann in der Form v[0], v[1], v[2] zugegriffen werden (anstatt v.x, v.y, v.z)
    //Jeweils ein Accessor für konstante und nicht konstante Objekte
    const T& operator [] (int i) const;
    T& operator [] (int i);
    //Vektoren addieren (Operator + überladen)
    Vec3<T> operator + (const Vec3<T> &v) const;
    //Vektoren subtrahieren (Operator - überladen)
    Vec3<T> operator - (const Vec3<T> &v) const;
    //Vektoren mit Skalar multiplizieren (Operator * überladen)
    //Skalar muss bei Multiplikation rechts vom Vektor stehen
    Vec3<T> operator * (const T &r) const;
};

//KONSTRUKTOREN

//Standardkonstruktor: Vektorobjekt ohne Parameter erzeugen
//x, y und z werden als 0 initialisiert
template<typename T>
Vec3<T>::Vec3() : x(T(0)), y(T(0)), z(T(0)) {}
//Vektorobjekt mit nur einem Parameter erzeugen
//Übergebener Wert wird dann zu x, y und z
template<typename T>
Vec3<T>::Vec3(const T &xx) : x(xx), y(xx), z(xx) {}
//Vektorobjekt mit übergebenen Parametern erzeugen
template<typename T>
Vec3<T>::Vec3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) {}

//KLASSENMETHODEN

//Betrag des Vektors bzw des Ortsvektors des Punktes berechnen
template<typename T>
T Vec3<T>::betrag() const
{
    return sqrt((x * x) + (y * y) + (z * z));
}
//Skalarprodukt mit einem übergebenen Vektor berechnen
template<typename T>
T Vec3<T>::skalarprod(const Vec3<T> &v) const
{
    return (x * v.x) + (y * v.y) + (z * v.z);
}
//Vektorprodukt (= Kreuzprodukt) mit einem übergebenen Vektor berechnen
template<typename T>
Vec3<T> Vec3<T>::vektorprod(const Vec3<T> &v) const
{
    return Vec3<T>
    (
        y * v.z - z * v.y,
        z * v.x - x * v.z,
        x * v.y - y * v.x
    );
}
//Vektor normalisieren
//Dazu müssen alle Komponenten des Vektors durch den Betrag geteilt werden
//Der Vektor wird nur normalisiert, wenn der Betrag > 0 ist (um Division durch 0 zu vermeiden)
//Anstatt alle Komponenten des Vektors durch den Betrag zu teilen (drei Divisionen),
//wird ein Faktor berechnet, mit dem die einzelnen Komponenten multipliziert werden (nur eine Division)
//Erhöht die Geschwindigkeit der Ausführung
template<typename T>
Vec3<T> Vec3<T>::normalisieren_neu() const
{
    //Betrag des Vektors bestimmen
    T betr = betrag();
    //Wenn der Vektorbetrag > 0 ist
    if (betr > 0)
    {
        T invLen = 1 / betr;
        return Vec3<T>
        (
            x * invLen,
            y * invLen,
            z * invLen
        );
    }
    //Wenn der Vektorbetrag = 0 ist, mit 0,0,0 initialisieren
    else
    {
        return Vec3<T>(0,0,0);
    }
}
//Vektor normalisieren
//Überschreiben der ursprünglichen Komponenten!
template<typename T>
Vec3<T>& Vec3<T>::normalisieren()
{
    //Betrag des Vektors bestimmen
    T betr = betrag();
    if (betr > 0)
    {
        T invLen = 1 / betr;
        x *= invLen;
        y *= invLen;
        z *= invLen;
    }
    return *this;
}
//Ausgabe der Vektorkoeffizienten als String für den seriellen Monitor
template<typename T>
String Vec3<T>::printVec3() const
{
  String strg_ausgabe = String('(');
  String strg1 = String(", ");
  strg_ausgabe += x;
  strg_ausgabe += strg1;
  strg_ausgabe += y;
  strg_ausgabe += strg1;
  strg_ausgabe += z;
  strg_ausgabe += ')';
  
  return strg_ausgabe;
}

//ÜBERLADENE OPERATOREN

//Access Operator (Accessor) (Operator [] überladen)
template<typename T>
const T& Vec3<T>::operator [] (int i) const
{
    return (&x)[i];
}
template<typename T>
T& Vec3<T>::operator [] (int i)
{
    return (&x)[i];
}
//Vektoren addieren (Operator + überladen)
template<typename T>
Vec3<T> Vec3<T>::operator + (const Vec3<T> &v) const
{
    return Vec3<T>
    (
        x + v.x,
        y + v.y,
        z + v.z
    );
}
//Vektoren subtrahieren (Operator - überladen)
template<typename T>
Vec3<T> Vec3<T>::operator - (const Vec3<T> &v) const
{
    return Vec3<T>
    (
        x - v.x,
        y - v.y,
        z - v.z
    );
}
//Vektoren mit Skalar multiplizieren (Operator * überladen)
template<typename T>
Vec3<T> Vec3<T>::operator * (const T &r) const
{
    return Vec3<T>
    (
        x * r,
        y * r,
        z * r
    );
}

//Neuen Datentyp definieren mit Klasse Vec3 als double
typedef Vec3<double> Vec3d;
//Neuen Datentyp definieren mit Klasse Vec3 als int
//typedef Vec3<int> Vec3i;

#endif
