/******************************************************************************

   Dateiname         : abstechn.c
   Erstellt von/am   : Karsten Paulini / 04.11.93


   Aenderungen:
   ------------

     Datum  |  Bearbeiter  |          Aenderung / betreffende Funktionen
   --------------------------------------------------------------------------
   10.01.94 |     kp       | "abstechen" geaendert: Es wird erst getestet, ob
            |              | auch etwas abgestochen wird.
   11.01.94 |     kp       | verlorenen Speicher freigegeben
   18.01.94 |     kp       | "kantenbruchweg" absturzsicher gemacht
   02.02.94 |     kp       | Konturnamen
   18.02.94 |     kp       | static-Funktionen
   19.05.94 |     kp       | abstechen_get_startx und abstechen_get_endx
   20.05.94 |     kp       | Berechnungsgenauigkeit fuer das startx erhoeht
            |              | Dazu wird temporaer die Iterationsgeanuigkeit des
            |              | G0-Crashtests erhoeht und hinterher wieder auf den
            |              | alten Wert gesetzt.
   05.07.94 |     kp       | veraenderte Schnittstelle zum Verfahrmodul
   28.09.94 |     kp       | Fase und Verrundung werden nicht mehr mit Hilfe
            |              | der entsprechenden GKE-Routinen berechnet.
   14.12.94 |     kp       | Routine "abstechen" faehrt beim Zurueckfahren
            |              | jetzt im G0 bis zum ev. auftretenden Crashpunkt
            |              | und den Rest im G1.
   23.03.95 |     kp       | Schnittstelle zu verfahr_init/exit neu
   11.09.01 | AC           | einstechen() aus abstechen() abgeleitet
   06.02.08 | JM           | (8896) Fehler bei Satzerzeugung 
                             und Crashliste != NULL -> Crashliste an Verfahrwege anhaengen
   07.03.08 | JM           | (8846) Der erste Verfahrweg mus um sr korrigiert werden.
            |              |


******************************************************************************/

#include <abstechn.h>
#include <elemente.h>
#include <box.h>
#include <basemath.h>
            Hallo
#include <schneide.h>
#include <schnitt.h>
#include <konturop.h>
#include <ncform.h>
#include <verfahr.h>
#include <g0verbin.h>



/*****************************************************************************/
/* Berechnet die Ausdehnung der Werkstueckkontur ueber dem angegebenen Z-Wert. */

static REAL ausdehnung ( kontur_p kp, REAL z )
{
    REAL            ret;
    punkt_t         ap;
    list_p          slistep = NULL;
    schnitt_info_p  infop;

    pkt2_set( &ap, z, -10.0 );
    mit_strahl_schneiden( ap, PI_HALBE, kp, &slistep, TRUE, FALSE );

    if ( infop = list_get_last(slistep) )
        ret = SI_GET_SCHNITTPUNKT(infop).y;
    else
        ret = 0.0;

    list_delete( slistep, TRUE );
    return ret;
}

/*****************************************************************************/
/* KP, 28.09.94 : Es wird nicht mehr die Fasen-Routine von GKE verwendet. */

element_p fasenweg ( kontur_p   rohp,
                     REAL       z,
                     REAL       breite,
                     BOOL       ri )    /* TRUE = Abstechen links von z */
{
    element_p       retp = NULL;
    schnittpunkt_t  sp;
    REAL            ax, ay, ex, ey;

    ax = ri ? z+breite : z-breite;
    ay = ausdehnung( rohp, ax );

    if ( !IS_ZERO_R(ay) )
    {
        ex = ri ? ax-1.0 : ax+1.0;
        ey = ay-1.0;
        schneide_gerade_gerade( ax, ay, ex, ey, z, 0.0, z, 1.0, &sp );

        if ( IS_GREATER_R( sp.y, 0.0) )
            retp = el_create_strecke_xy( ax, ay, z, sp.y );
    }

    return retp;
}

/*****************************************************************************/
/* KP, 28.09.94 : Es wird nicht mehr die Fasen-Routine von GKE verwendet. */

element_p verrundungsweg ( kontur_p rohp,
                           REAL     z,
                           REAL     radius,
                           BOOL     ri )    /* TRUE = Abstechen links von z */
{
    
    element_p       retp = NULL;
    REAL            ax, ay, ex, ey, mx, my;

    ax = ri ? z+radius : z-radius;
    ay = ausdehnung( rohp, ax );

    if ( !IS_ZERO_R(ay) )
    {
        mx = ax;
        my = ey = ay-radius;
        ex = z;

        if ( IS_GREATER_R( my, 0.0) )
            retp = el_create_bogen_xy( ax, ay, ex, ey, mx, my, ri ? CCW : CW );
    }

    return retp;
}

/*****************************************************************************/
/* Erzeugt ein Verfahrelement zur Erzeugung des in elp uebergebenen Kanten-
   bruchelementes. Das Verfahrelement steuert den Werkzeugmittelpunkt. */

static element_p kantenbruchweg_korrigieren ( element_p elp, REAL sr, REAL mb )
{
    element_p   retp = el_bilde_aufmass( elp, sr, -1);

    // Wenn das Element eine Strecke ist (also der Kantenbruch eine Fase),
    // muss das um den Schneidenradius korrigierte Element nach beiden
    // Seiten etwas verlaengert werden !
    // Im Fall einer Verrundung ist nichts mehr zu tun.

    if ( EL_TYP(retp)==EL_STRECKE )     // der Kantenbruch ist eine Fase
    {
        punkt_t         p1, p2, p3;
        schnittpunkt_t  s1, s2;

        p1 = EL_GET_AP( retp );         // p1 und p2 bilden die Traegergerade
        p2 = EL_GET_EP( retp );         // des Aufmasselementes
        p3 = EL_GET_AP( elp );
        schneide_gerade_gerade( p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p3.x, p3.y+10.0, &s1 );

        p3 = EL_GET_EP( elp );
        schneide_gerade_gerade( p1.x, p1.y, p2.x, p2.y, p3.x-sr, p3.y, p3.x-sr, p3.y+10.0, &s2 );

        p1.x = s1.x; p1.y = s1.y;       // Kompatibilitaet herstellen
        p2.x = s2.x; p2.y = s2.y;
        el_init_strecke( retp, &p1, &p2 );
    }

    // Damit ist der Schneidenradius ausgeglichen. Zum Schluss muss noch
    // die Meisselbreite korrigiert werden. Das erzeugte Element bezieht
    // sich auf den Mittelpunkt des Stechmeissels.

    el_verschiebe ( retp, -(mb/2.0-sr), 0.0 );
    return retp;
}

/*****************************************************************************/
/* Erzeugt eine senkrechte Verfahrbewegung auf den Radius x. */

static void im_g1_auf ( REAL x, ncflib_vorschub_t vorschub )
{
    punkt_t p = get_akt_position();

    p.y = x;
    g1_absolut( p, vorschub );
}

/*****************************************************************************/
/* Erzeugt die fuer die Anbringung des Kantenbruchs (Fase/Verrundung) in ktbp
   noetigen Verfahrwege. Es wird davon ausgegangen, dass der Kantenbruch mit
   der rechten Schneide des Stechmeissels erzeugt werden kann.
   In dx/dy wird der SRK-Offset des Werkzeugs uebergeben.
   Kann der Startpunkt nicht im G0 erreicht werden, wird FALSE zurueckgegeben.
    
    07.03.08 | JM | (8846) Der erste Verfahrweg mus um sr korrigiert werden.
*/
static BOOL kantenbruch_erzeugen ( element_p    ktbp,
                                   REAL         sr,
                                   REAL         mb,
                                   REAL         dx,
                                   REAL         dy )
{
    BOOL        verfahren;
    punkt_t     p1, p2;
    kontur_p    wegep = KONTUR_CREATE("abstechn_kantenbruch_erzeugen");
    element_p   elp = kantenbruchweg_korrigieren( ktbp, sr, mb );

    // Zuerst in X auf den Endradius der Schablone (senkrecht im G1) :

    im_g1_auf( EL_GET_EY(elp)+ dy, NORMALVORSCHUB );

    // Dann den Kantenbruch um senkrechte Anfahrbewegung erweitern (2 mm) ...

    p2 = EL_GET_AP(elp);
    p1 = p2;
    p1.y += 2.0;
    kontur_insert_first( wegep, el_create_strecke( &p1, &p2) );
    kontur_insert_last( wegep, elp );

    // ... die Verfahrwege vom Werkzeugmittelpunkt auf den P-Punkt beziehen :

    kontur_verschiebe_um( wegep, dx, dy);
    kontur_get_start( wegep, &p1);

    // ... den Startpunkt im G0 anfahren und den Kantenbruch abfahren :

    if ( verfahren = mit_g0_verbinden(p1) )
        kontur_abfahren( wegep );

    kontur_delete( wegep, TRUE );   // hier wird elp mitgeloescht !
    return verfahren;
}

/*****************************************************************************/
/* Testet, ob bei der durch z, x1, x2 beschriebenen Bewegung mit dem Werkzeug
   etwas vom Rohteil abgestochen wird. rohp und wkzp muessen !=NULL sein ! */

static BOOL abstechbewegung ( kontur_p  rohp,
                              wkz_geo_p wkzp,
                              REAL      z,
                              REAL      x1,
                              REAL      x2 )
{
    element_p   strp;
    kontur_p    schneidep, cutp;
    list_p      cutlistp;
    BOOL        ret = FALSE;

    if ( IS_EQUAL_R( x1, x2) )
        return FALSE;

    strp = el_create_strecke_xy( z, x1, z, x2 );
    schneidep = KONTUR_CREATE("Schneidenkontur");
    wkz_get_geschlossene_schneiden_schneide( wkzp, schneidep );
    initcut();
    cutlistp = get_move_path_xy( strp, schneidep );
    exitcut();

    if( cutlistp)
    {
        cutp = list_get_first( cutlistp );
        while ( cutp && !ret )
        {
            ret = ueberlappungstest( rohp, cutp );
            cutp = LIST_GET_NEXT( cutlistp );
        }
    }

    KONTUR_DELETE( schneidep );
    delete_liste_von_konturen( cutlistp, TRUE );
    el_free( strp );

    return ret;
}

/*****************************************************************************/
/* Erzeugt die zum Abstechen des Werkstuecks bei z noetigen Verfahrbewegungen.
   Das Werkzeug muss ein Aussenstechmeissel sein. ktbp ist ein von "fasenweg"
   oder "verrundungsweg" berechnetes Kantenbruchelement. Wird NULL uebergeben,
   wird kein Kantenbruch beim Abstechen eingefuegt.
   Fuer die zu uebergebenden Radien x1, x2 und x3 muss folgendes gelten :
   x1 > x2 >=x3 und x3<=0 !
   Eine ev. anzubringende Fase oder Verrundung muss oberhalb von x2 liegen.
   Gilt x2=x3, so wird kein Technologiewechsel beruecksichtigt.
   Fuer die erzeugten Verfahrwege (ncforms) gilt folgende Konvention :
   Ist ein Uebergangsradius definiert, so sind die mit einer alternativen
   Technologie zu fahrenden Saetze in der Liste markiert, sonst nicht. */

int abstechen ( kontur_p    rohp,               // aktuelles Rohteil
                kontur_p   *neurohp,            // aufdatiertes Rohteil nach dem ABS
                wkz_geo_p   wkzp,               // muss ein Stechmeissel sein !
                REAL        z,                  // Abstechposition in Z
                REAL        x1,                 // Startradius
                REAL        x2,                 // Uebergangsradius (andere Technologie)
                REAL        x3,                 // Endradius (<=0)
                element_p   ktbp,               // Kantenbruchweg (Fase/Rundung/nichts)
                list_p     *verfahrweglistep,   // erzeugte Wege
                punkt_t    *startwkzpos,        // Position vor dem ABS
                punkt_t    *endwkzpos )         // Position nach dem ABS
{
    return einstechen( rohp,
                       neurohp,
                       wkzp,
                       z,
                       x1,
                       x2,
                       x3,
                       ktbp,
                       NULL,
                       verfahrweglistep,
                       startwkzpos,
                       endwkzpos);
}

/*****************************************************************************/
/* einstechen() aus abstechen() abgeleitet.
   einstechen() besitzt im Gegensatz zu abstechen() Kantenbruch-Elemente auf
   beiden Seiten. Die Stechrichtung ist fest von oben nach unten angelegt. */

int einstechen( kontur_p    rohp,               // aktuelles Rohteil
                kontur_p   *neurohp,            // aufdatiertes Rohteil nach dem ABS
                wkz_geo_p   wkzp,               // muss ein Stechmeissel sein !
                REAL        z,                  // Abstechposition in Z
                REAL        x1,                 // Startradius
                REAL        x2,                 // Uebergangsradius (andere Technologie)
                REAL        x3,                 // Endradius (<=0)
                element_p   kante_r,            // Kantenbruchweg rechts (Fase/Rundung/nichts)
                element_p   kante_l,            // Kantenbruchweg links (Fase/Rundung/nichts)
                list_p     *verfahrweglistep,   // erzeugte Wege
                punkt_t    *startwkzpos,        // Position vor dem ABS
                punkt_t    *endwkzpos )         // Position nach dem ABS
{
    punkt_t ap;
    BOOL    verfahren;
    REAL    mb, sr;     // Meisselbreite, Schneidenradius
    REAL    dx, dy;     // SRK-Offset
    int     vstatus = ABSTECHEN_OK;
    list_p  vlistp  = verfahr_init( rohp, wkzp, NULL, 0 );

    // KP, 10.01.94 : Erst testen, ob auch etwas abgestochen wird !

    if ( rohp && wkzp && abstechbewegung( rohp, wkzp, z, x1, x3) )
    {
        // Umrechnung der Eingangsdaten auf den Programmierpunkt des Wkz.
        // Dazu erst alles auf den Mittelpunkt rechnen, dann um SRK-Offset verschieben.
        // Der so ermittelte Startpunkt wird im G0 angefahren.

        mb = wkz_get_schneiden_breite( wkzp);
        wkz_get_schneidenradius( wkzp, &sr);
        wkz_get_srk_offset( wkzp, &dx, &dy );
        pkt2_set( &ap, z-mb/2.0+dx, x1+sr+dy );
        verfahren = mit_g0_verbinden(ap);

        // ggf. Kantenbrueche erzeugen :

        if( verfahren && kante_r)
            verfahren = kantenbruch_erzeugen( kante_r, sr, mb, dx, dy);

        if( verfahren && kante_l)
            verfahren = kantenbruch_erzeugen( kante_l, -sr, -mb, dx, dy);

        // Bis zum Uebergangsradius fahren (ev. gleich dem Endradius) :


        if ( verfahren )
            im_g1_auf( x2, NORMALVORSCHUB );

        // Weiter auf den Endradius fahren (wenn noch nicht erreicht) :

        if ( verfahren && !IS_EQUAL_R( x2, x3) )
            im_g1_auf( x3, VORSCHUB1 );

        // Dann wieder zum Ausgangspunkt zurueckkehren :

        if ( verfahren )        /* KP, 14.12.94 : */
        {
            element_t   teststrecke;
            punkt_t     aktpos;
            punkt_t     crashpunkt;

            aktpos = get_akt_position();
            el_init_strecke( &teststrecke, &aktpos, &ap );
            get_crashpunkt( &teststrecke, 0, &crashpunkt );

            if ( !pkt2_is_undef(&crashpunkt) )
            {
                g0_absolut( crashpunkt );
                g1_absolut( ap, NORMALVORSCHUB );
            }
            else
            {
                g0_absolut( ap );
            }
        }
    }

    // Ergebnisdaten auslesen :

    loesche_fuehrende_g0( vlistp );
    verfahr_exit( neurohp, NULL, &vstatus );
    if ( !verfahren && (vstatus==ABSTECHEN_OK) )
        vstatus = ABSTECHEN_G0_ERROR;
    *verfahrweglistep = vlistp;
    
    if( vstatus!= ABSTECHEN_OK)
        g0_verbindung_add_crashlist( *verfahrweglistep, CRASHLIST_MODE_AUTO);
    g0_verbindung_exit_crashlist();
    get_start_und_endposition( *verfahrweglistep, startwkzpos, endwkzpos );

    return vstatus;
}

/*****************************************************************************/
/* Schickt von den "Eckpunkten" des Stechmeissels (ein Schneidenradius wird
   ignoriert) Teststrahlen gegen die Kontur. Die zuerst gefundenen Schnitt-
   punkte werden, wenn vorhanden, zurueckgegeben, sonst stehen sie auf Undef.
   Der BOOL-Parameter gibt an, ob die Strahlen von unten nach oben oder von
   oben nach unten geschossen werden.
   Der uebergebene z-Wert bezieht sich auf die z-Position des Mittelpunktes
   des Stechmeissels. */

static void teststrahlen_schiessen ( kontur_p    rohp,
                                     REAL        mb,
                                     REAL        z,
                                     BOOL        nach_oben,
                                     punkt_p     sp1p,
                                     punkt_p     sp2p )
{
    REAL            z1 = z + mb/2.0;    /* Z-Korrektur fuer die "Meisselecken" */
    REAL            z2 = z - mb/2.0;
    REAL            x1, x2;
    REAL            winkel;
    punkt_t         ap1, ap2;
    list_p          sliste1p, sliste2p;
    schnitt_info_p  infop;

    if ( nach_oben )
    {
        x1 = x2 = -10.0;
        winkel = PI_HALBE;
    }
    else
    {
        punkt_t p1, p2;

        box_get_ap_ep( &rohp->box, &p1, &p2 );
        x1 = x2 = p2.y+10.0;
        winkel = 3*PI_HALBE;
    }

    ap1.x = z1;
    ap2.x = z2;
    ap1.y = x1;
    ap2.y = x2;

    sliste1p = sliste2p = NULL;
    mit_strahl_schneiden( ap1, winkel, rohp, &sliste1p, TRUE, FALSE );
    mit_strahl_schneiden( ap2, winkel, rohp, &sliste2p, TRUE, FALSE );

    if ( infop = list_get_first(sliste1p) )
        *sp1p = SI_GET_SCHNITTPUNKT(infop);
    else
        pkt2_set_undef(sp1p);

    if ( infop = list_get_first(sliste2p) )
        *sp2p = SI_GET_SCHNITTPUNKT(infop);
    else
        pkt2_set_undef(sp2p);

    list_delete( sliste1p, TRUE);
    list_delete( sliste2p, TRUE);
}

/*****************************************************************************/
/* Berechnet den Startradius der Abstechbewegung. Dazu wird auf dem korri-
   gierten Z-Wert eine Teststrecke von oben nach unten konstruiert und der
   G0-Crashpunkt des Wkz entlang dieser Strecke ermittelt. Der gefundene
   y-Wert wird zurueckgegeben.
   Die Genauigkeit der ermittelten Werte haengt somit von der Genauigkeit
   der G0-Crashpunktermittlung ab. Diese wird zur Zeit iterativ durchge-
   fuehrt. */

BOOL abstechen_get_startx ( kontur_p    rohp,   /* aktuelles Werkstueck */
                            wkz_geo_p   wkzp,   /* aktuelles Werkzeug */
                            REAL        z,      /* Abstechposition in Z */
                            REAL       *x )     /* Startradius */
{
    BOOL        ret = FALSE;
    REAL        mb, sr, dx, dy, c_eps;
    REAL        ax, ex, ay, ey;
    punkt_t     crashpunkt, p1, p2;
    element_t   strecke;

    if ( rohp && !kontur_is_empty(rohp) && wkzp )
    {
        mb = wkz_get_schneiden_breite( wkzp);
        wkz_get_schneidenradius( wkzp, &sr);
        z -= mb/2.0;

        teststrahlen_schiessen( rohp, mb, z, FALSE, &p1, &p2);

        ey = -10.0;
        if ( IS_DEF_R(p1.y) )
            ey = MAX( ey, p1.y);
        if ( IS_DEF_R(p2.y) )
            ey = MAX( ey, p2.y);

        /* Teststrecke fuer den Programmierpunkt konstruieren :
           Dazu werden die Koordinaten erst mal fuer "unseren" Mittelpunkt
           des Stechmeissels konstruiert und dann mit dem Offset in den
           Programmierpunkt geschoben. */

        ax = ex = z;
        ay = rohp->box.ap.y + rohp->box.size.h + 10.0 + sr;
        ey += sr;
        wkz_get_srk_offset( wkzp, &dx, &dy );
        ax += dx;
        ex += dx;
        ay += dy;
        ey += dy;

        /* Um die Verlaengerung der G0-Wege beim Kollisionstest zu umgehen,
           wird der Endpunkt der Teststrecke noch ein Stueck hochgezogen.
           Dieses Stueck muss mindestens so lang sein, wie die G0-Verlaengerung. */

        ey += 0.06;
        el_init_strecke_xy( &strecke, ax, ay, ex, ey );

        /* Fuer den Kollsionstest hier brauchen wir eine relativ hohe Genauigkeit. */

        c_eps = get_crasheps();
        set_crasheps( 0.1 );

        if ( crashpunkt_g0_1( rohp, NULL, wkzp, &strecke, &crashpunkt) )
        {
            *x  = crashpunkt.y;
            ret = TRUE;
        }
        else
        {
            ey -= 0.06;     /* wieder abziehen */
            *x = ey;
            ret = !IS_EQUAL_R(ey, -10.0);
        }

        set_crasheps( c_eps );
    }

    return ret;
}

/*****************************************************************************/
/* Berechnet den Endradius der Abstechbewegung. Dazu wird das Wkz idealisiert
   als ein Stechmeissel ohne Schneidenradius angenommen. Von den beiden Eck-
   punkten (bzw. auf ihrem Z-Wert) werden Strahlen senkrecht nach oben
   geschossen. Die y-Koordinate des zuerst gefundenen Schnittpunktes wird
   zurueckgegeben.
   Dieses Verfahren liefert i.a. natuerlich nicht den richtigen Wert. Aber die
   so ermittelten Werte genuegen auf jeden Fall der Forderung, dass das
   Werkstueck damit abgestochen werden kann.  */

BOOL abstechen_get_endx   ( kontur_p    rohp,   /* aktuelles Werkstueck */
                            wkz_geo_p   wkzp,   /* aktuelles Werkzeug */
                            REAL        z,      /* Abstechposition in Z */
                            REAL       *x )     /* Endradius */
{
    BOOL    ret = FALSE;
    REAL    mb;
    punkt_t sp1, sp2;

    if ( rohp && !kontur_is_empty(rohp) && wkzp )
    {
        mb = wkz_get_schneiden_breite( wkzp);
        z  = z-mb/2.0;
        teststrahlen_schiessen( rohp, mb, z, TRUE, &sp1, &sp2);

        *x = -REAL_UNDEF;
        if ( IS_DEF_R(sp1.y) )
            *x = MAX( *x, sp1.y );
        if ( IS_DEF_R(sp2.y) )
            *x = MAX( *x, sp2.y );

        ret = IS_DEF_R(*x);
    }

    return ret;
}

/*****************************************************************************/

