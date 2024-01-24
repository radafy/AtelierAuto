#include <stdio.h>

// Déclaration de la fonction
int sommeDeuxNombres() {
    // Déclaration des variables
    int nombre1, nombre2, somme;

    // Demande à l'utilisateur d'entrer le premier nombre
    printf("Entrez le premier nombre : ");
    scanf("%d", &nombre1);

    // Demande à l'utilisateur d'entrer le deuxième nombre
    printf("Entrez le deuxième nombre : ");
    scanf("%d", &nombre2);

    // Calcul de la somme
    somme = nombre1 + nombre2;

    // Affichage du résultat
    printf("La somme de %d et %d est : %d\n", nombre1, nombre2, somme);

    // Retourne la somme
    return somme;
}

int main() {
    // Appel de la fonction
    sommeDeuxNombres();

    return 0;
}
