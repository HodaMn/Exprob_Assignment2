

(define (domain exp_domain)

;remove requirements that are not needed
(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types 
    robot
    waypoint
    marker
)

(:predicates 
    (robot_at ?r - robot ?wp - waypoint)
    (marker_at ?mk - marker ?wp - waypoint)
    (detected ?mk - marker)
    (not_detected ?mk - marker)
    (visited ?wp - waypoint)
    (not_visited ?wp - waypoint)
    (all_markers_detected)
    
)

;actions
(:action go_to_waypoint
    :parameters (?r - robot ?from ?to - waypoint)
    :precondition (and 
                    (robot_at ?r ?from)
                    (not_visited ?to)                    
    )
    
    :effect (and                
                (not (not_visited ?to))
                (visited ?to)
                (robot_at ?r ?to)
                (not (robot_at ?r ?from))
    )
)

(:action detect
    :parameters (?r - robot ?wp - waypoint ?mk - marker)
    :precondition (and 
                    (robot_at ?r ?wp)
                    (marker_at ?mk ?wp)
                    (not_detected ?mk)
    )
    
    :effect (and                
                (not (not_detected ?mk))
                (detected ?mk)
    )
)

(:action detect_all_markers ; checking if all markers are detected
    :parameters ()
    :precondition (and 
                    (forall (?mk - marker) (detected ?mk)) ; all markers detected
    )
    :effect (and 
                (all_markers_detected) 
    )
)
)
