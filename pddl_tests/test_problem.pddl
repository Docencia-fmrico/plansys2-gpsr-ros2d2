(define (problem test_problem)
(:domain test_domain)

(:objects 
  bathroom bedroom kitchen livingroom - room
  D1 D2 D3 - door
  D1_k D1_lr D2_br D2_lr D3_bd D3_bath - predoor
  towel dish glass - object
  r2d2 - robot
  granny - person
)

(:init
  (robot_at r2d2 livingroom)
  (robot_available r2d2)
  (person_at granny kitchen)

  (object_at towel livingroom)
  (object_at dish bathroom)
  (object_at glass bathroom)

  (connected kitchen D1_k)
  (connected D1_k kitchen)
  (connected livingroom D1_lr)
  (connected D1_lr livingroom)
  (door_connection D1 D1_k D1_lr)
  (door_connection D1 D1_lr D1_k)
  (next_to_door D1_k D1)
  (next_to_door D1_lr D1)

  (connected bedroom D2_br)
  (connected D2_br bedroom)
  (connected livingroom D2_lr)
  (connected D2_lr livingroom)
  (door_connection D2 D2_br D2_lr)
  (door_connection D2 D2_lr D2_br)
  (next_to_door D2_br D2)
  (next_to_door D2_lr D2)

  (connected bedroom D3_bd)
  (connected D3_bd bedroom)
  (connected bathroom D3_bath)
  (connected D3_bath bathroom)
  (door_connection D3 D3_bd D3_bath)
  (door_connection D3 D3_bath D3_bd)
  (next_to_door D3_bd D3)
  (next_to_door D3_bath D3)

  (door_open D1)
  (door_closed D2)
  (door_open D3)

  ; Human orders

  ; Active type of orders from the human
  (no_object_request r2d2)
  (no_open_door_request r2d2)
  (no_close_door_request r2d2)
)

(:goal (and
    (robot_at r2d2 kitchen)
  )
)

)