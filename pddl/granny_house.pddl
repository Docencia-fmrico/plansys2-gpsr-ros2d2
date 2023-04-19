(define (domain granny_house)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
room
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(battery_full ?r - robot)
(robot_available ?r - robot)
(robot_at ?r - robot ?z - room)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;

;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;

;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?z1 ?z2 - room)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?z1))
        (at start(robot_available ?r))
        )
    :effect (and
        (at start(not(robot_at ?r ?z1)))
        (at end(robot_at ?r ?z2))
        (at start(not(robot_available ?r)))
        (at end(robot_available ?r))
    )
);; end Actions ;;;;;;;;;;;;;;;;;;;;;;

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
