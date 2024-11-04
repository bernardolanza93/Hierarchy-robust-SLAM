import numpy as np
from decorator import *
from fusion_utilities import *
import open3d as o3d




@timeit
def hierarchy_slam_icp(input_all_pointcloud,timestamp_sorted):

    # devo saltare la zero che è solo target
    # prendo la sua trasformata trovata tra le sotto pc e la applico alla coppia dopo
    # current_source.trasform(last_epoch_trasformation[i-1])
    # NDR la trasformazione [i-esima] l ho gia usata l epoca prima per traslare la PC
    # tecnicamente cosi ho PC 0,1,2,3  0-1 2-3 gia fuse, applicando questa nuova tresformazione in pratica sto sovrapponendo pc2 a pc1 cosi avremo in serie 0 1+2 e 3
    # la trasformazione nuova mi farà avanzare 2-3 alla fine di 1,
    # ogni iterazione individui la trasformazione 1-2 che però potrebbe crescere con le iterazioni (dipende na quante epoche hanno costruito 1)
    # se volessi avvicinarle ancora dovrei sommare la trasformazione 2-3 anche, (da rivedere perche potrebbe essere troppo)
    # fuse PC[0],PC[1]
    # fuse PC[i],PC[i+1]
    # trasformation_estimated = icp
    # trasfoirmation_current.append(trasformation_estimated)
    # source-trasform to match target ->(trasformation_estimated)
    # fuse target and source transformed
    # new_matrices.append(fusion)

    # al primo giro prendo PC grezze
    epoch_start_pointclouds = input_all_pointcloud
    epoch = 0

    last_epoch_trasformation = [np.eye(4) for _ in range(len(input_all_pointcloud))]
    x_y = []
    timestamp_dict = {}

    while len(epoch_start_pointclouds) > 1:
        start_time = time.time()
        epoch += 1
        i = 1

        new_halfed_pointcloud = []
        trasformation_current = []

        print(f"START Epoch {epoch}, PCs:{len(epoch_start_pointclouds)}")


        while i < len(epoch_start_pointclouds):


            if len(epoch_start_pointclouds) == 2:
                print("LAST EPOCH")
                print(len(last_epoch_trasformation))
                print(len(epoch_start_pointclouds))

            #TOGLILO
            if i > 0:

                #print(i, end=" ")

                initial_trasform_from_last_epoch_1_couple = (last_epoch_trasformation[i-1])
                initial_trasform_from_last_epoch_2_couple = (last_epoch_trasformation[i])

                # CONSIDERO I PRIOR DI ENTRAMBE LE SOURCE PASSATE, PERCHE SE NO PIU AUMENTANO LE EPOCE PIU AUMENTA IL LAG TRA LE POINTCLOUS.
                # QUINDI PRENDO IL PRIOR TRASFORMATION DA ENTRAMBE LE PC CHE HANNO COSTRUITO LA CORRENTE.



                #Salvolo entrambe però perche l iterazione dopo saranno lunghe il doppio e mi servve la lunghezza di una piu l altra per traslare
                prior_trasformation_composed = np.dot(initial_trasform_from_last_epoch_1_couple, initial_trasform_from_last_epoch_2_couple)
                #prior_trasformation_composed = initial_trasform_from_last_epoch_2_couple

                source_raw = epoch_start_pointclouds[i]

                current_source = o3d.geometry.PointCloud(source_raw)
                # devo aggiungere anche la trasformata delle iterazioni prima BASTA CHE LE LAST EPOCH LA COMPRENDANO

                current_source.transform(initial_trasform_from_last_epoch_1_couple)

                target =  epoch_start_pointclouds[i-1]


                VOXEL_VOLUME = 0.03
                current_source = downsample_point_cloud(current_source, VOXEL_VOLUME)
                target = downsample_point_cloud(target, VOXEL_VOLUME)



                SHOW_ICP_PROCESS = 0
                if SHOW_ICP_PROCESS:
                    if len(current_source.points) < 600 or len(target.points) < 600:
                        target = remove_isolated_points(target, nb_neighbors=12, radius=0.7)
                        current_source = remove_isolated_points(current_source, nb_neighbors=12, radius=0.8)

                    yellow = np.array([1, 1, 0])
                    red =  np.array([1, 0, 0])
                    green = np.array([0, 1, 0])
                    blue = np.array([0, 0, 1])

                    # print("source",len(current_source.points))
                    # print("target", len(target.points))
                    if len(current_source.points) < 600 or len(target.points) < 600:
                        target = remove_isolated_points(target, nb_neighbors=12, radius=0.7)
                        current_source = remove_isolated_points(current_source, nb_neighbors=12, radius=0.8)

                    current_source.colors = o3d.utility.Vector3dVector(np.tile(yellow, (len(current_source.points), 1)))
                    target.colors = o3d.utility.Vector3dVector(np.tile(red, (len(target.points), 1)))
                    pre_icp = target + current_source
                    visualize_pc(pre_icp, "pre icp")

                    # Imposta tutti i punti al colore bianco (RGB: [1, 1, 1])
                    white = np.array([1, 1, 1])
                    current_source.colors = o3d.utility.Vector3dVector(np.tile(green, (len(current_source.points), 1)))
                    target.colors = o3d.utility.Vector3dVector(np.tile(blue, (len(target.points), 1)))






                updated_trasform_icp_result = icp_open3d(current_source, target, 0.05, 200)
                #Total trasformation : current, plus the prrevious frame trasf:
                x_y.append(updated_trasform_icp_result[0, -3:])
                total_trasformation_prior_plus_icp = np.dot(prior_trasformation_composed, updated_trasform_icp_result)


                trasformed_icp_source = o3d.geometry.PointCloud(current_source)
                trasformed_icp_source.transform(updated_trasform_icp_result)

                if len(epoch_start_pointclouds) == 2:
                    print(updated_trasform_icp_result)

                    # pre_icp  = target + current_source
                    # visualize_pc(pre_icp, "pre icp")


                merged_pcd = target + trasformed_icp_source





                if SHOW_ICP_PROCESS:
                    visualize_pc(merged_pcd, "merged")
                    current_source.colors = o3d.utility.Vector3dVector(np.tile(white, (len(current_source.points), 1)))
                    target.colors = o3d.utility.Vector3dVector(np.tile(white, (len(target.points), 1)))



                # Controllo se esiste una point cloud precedente e successiva
                # Controllo se esiste una point cloud precedente e successiva

                PREVIOUS_ADDICTION = 0
                if PREVIOUS_ADDICTION:
                    #all inizio non ha senso usarlo perche la percentuale di sovrapposizione è alta
                    if epoch > 4:
                        if i - 2 >= 0:
                            previous_pc = epoch_start_pointclouds[i - 2]
                            initial_transform_previous_1 = last_epoch_trasformation[i - 2]
                            initial_transform_previous_2 = last_epoch_trasformation[i - 1]
                            prior_transform_composed_previous = np.dot(initial_transform_previous_1,
                                                                       initial_transform_previous_2)

                            #COME AGGIORNATO BASTA UNA SOLA TRASFORMAZIONE
                            # Trasformare la merged_pcd
                            merged_pcd.transform(initial_transform_previous_1)
                            # Eseguire ICP per allineare merged_pcd con previous_pc
                            previous_pc_transformed = o3d.geometry.PointCloud(previous_pc)
                            updated_transform_icp_result_previous = icp_open3d(merged_pcd, previous_pc_transformed)

                            # Correggere merged_pcd con la trasformazione ICP trovata
                            merged_pcd.transform(updated_transform_icp_result_previous)
                            merged_pcd += previous_pc_transformed

                NEXT_ADDITION = 0
                if NEXT_ADDITION:


                    if i + 1 < len(epoch_start_pointclouds):
                        next_pc = epoch_start_pointclouds[i + 1]


                        #COME AGGIORNATO INVECE CHE 2 NE BASTA UNA INVECE CHE 6 NE BASTANO 3

                        # Coppia di trasformazioni per unire la previous
                        transform_previous_1 = last_epoch_trasformation[i - 2]
                        transform_previous_2 = last_epoch_trasformation[i - 1]
                        transform_composed_previous = np.dot(transform_previous_1, transform_previous_2)

                        # Coppia di trasformazioni per unire la current
                        transform_current_1 = last_epoch_trasformation[i]
                        transform_current_2 = last_epoch_trasformation[i + 1]
                        transform_composed_current = np.dot(transform_current_1, transform_current_2)

                        # Coppia di trasformazioni per unire la next (per la prossima iterazione)
                        transform_next_1 = last_epoch_trasformation[i]
                        transform_next_2 = last_epoch_trasformation[i + 1]
                        transform_composed_next = np.dot(transform_next_1, transform_next_2)

                        # Combinare tutte le trasformazioni
                        combined_transform = np.dot(transform_previous_1, transform_current_1)
                        combined_transform = np.dot(combined_transform, transform_next_1)

                        # Trasformare la merged_pcd con le trasformazioni combinate
                        next_pc.transform(combined_transform)

                        # Eseguire ICP per allineare merged_pcd con next_pc
                        next_pc_transformed = o3d.geometry.PointCloud(next_pc)
                        transform_icp_result_next = icp_open3d(merged_pcd, next_pc_transformed)

                        # Correggere merged_pcd con la trasformazione ICP trovata
                        next_pc_transformed.transform(transform_icp_result_next)
                        merged_pcd += next_pc_transformed


                merged_pcd = remove_isolated_points(merged_pcd, nb_neighbors=12, radius=0.4)
                trasformation_current.append(total_trasformation_prior_plus_icp)
                new_halfed_pointcloud.append(merged_pcd)


                # i = 97, len = 99,
                # quando i = 99 rompe e quindi perderò la pointclou i[98], che sarabbe il target di i[99] che non arriveà
                if i == len(epoch_start_pointclouds) - 2 and len(epoch_start_pointclouds) % 2 != 0:  # Vogliamo eliminare
                # SKip the odd last matrixprint
                   # 100/101 i  = 100

                    print("DISPARI")
                    print(f"START Epoch {epoch},i = {i}, PCs AVIABLE:{len(epoch_start_pointclouds)} PCS comp:{len(new_halfed_pointcloud)} trasform:{len(trasformation_current)}")
                    print("add last PC with its trasformation")
                    # TO DO TO INCLUDE LAST
                    last_pc = epoch_start_pointclouds[i+1] #98
                    last_trasaform = (last_epoch_trasformation[i+1]) #98

                    trasformation_current.append(last_trasaform)
                    new_halfed_pointcloud.append(last_pc)

                timestamp_index = i // 2 if epoch == 1 else len(timestamp_sorted) // (2 ** (epoch - 1)) + i // 2
                timestamp_dict[timestamp_sorted[timestamp_index]] = updated_trasform_icp_result

            i += 2


        epoch_start_pointclouds = new_halfed_pointcloud
        last_epoch_trasformation = trasformation_current
        end_time = time.time()
        elapsed_time = end_time - start_time
        elapsed_time_ms = round(elapsed_time, 3)
        print(f"Computed Epoch {epoch} in {elapsed_time_ms} s, PCs and T created:{len(new_halfed_pointcloud)}")


    # Estraiamo i valori di x e y
    x_values = [item[0] for item in x_y]
    y_values = [item[1] for item in x_y]
    z_values = [item[2] for item in x_y]




    plot_trajectory_3d(timestamp_dict)


    return new_halfed_pointcloud[0]