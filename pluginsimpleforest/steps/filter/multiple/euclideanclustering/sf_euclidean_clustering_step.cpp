#include "sf_euclidean_clustering_step.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

SF_Euclidean_Clustering_Step::SF_Euclidean_Clustering_Step(CT_StepInitializeData &data_init): SF_Abstract_Filter_Multiple_Step(data_init) {

}

SF_Euclidean_Clustering_Step::~SF_Euclidean_Clustering_Step() {

}

QString SF_Euclidean_Clustering_Step::getStepDescription() const {
    return tr("Euclidean Clustering");
}

QString SF_Euclidean_Clustering_Step::getStepDetailledDescription() const {
    return tr("Performs an Euclidean Clustering Operation.");
}

QString SF_Euclidean_Clustering_Step::getStepURL() const {
    return tr("");
}

CT_VirtualAbstractStep* SF_Euclidean_Clustering_Step::createNewInstance(CT_StepInitializeData &dataInit) {
    return new SF_Euclidean_Clustering_Step(dataInit);
}

QStringList SF_Euclidean_Clustering_Step::getStepRISCitations() const {
    QStringList _RIS_citation_list;
    _RIS_citation_list.append(QString("TY  - JOUR\n"
                                      "T1  - SimpleTree - an efficient open source tool to build tree models from TLS clouds\n"
                                      "A1  - Hackenberg, Jan\n"
                                      "A1  - Spiecker, Heinrich\n"
                                      "A1  - Calders, Kim\n"
                                      "A1  - Disney, Mathias\n"
                                      "A1  - Raumonen, Pasi\n"
                                      "JO  - Forests\n"
                                      "VL  - 6\n"
                                      "IS  - 11\n"
                                      "SP  - 4245\n"
                                      "EP  - 4294\n"
                                      "Y1  - 2015\n"
                                      "PB  - Multidisciplinary Digital Publishing Institute\n"
                                      "UL  - http://www.simpletree.uni-freiburg.de/\n"
                                      "ER  - \n"));
    return _RIS_citation_list;
}


void SF_Euclidean_Clustering_Step::createInResultModelListProtected() {
    CT_InResultModelGroupToCopy *res_model = createNewInResultModelForCopy(DEF_IN_RESULT, tr("Point Cloud"));
    //    assert(res_model != NULL);
    res_model->setZeroOrMoreRootGroup();
    res_model->addGroupModel("", DEF_IN_GRP, CT_AbstractItemGroup::staticGetType(), tr("Input Cloud Group"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    res_model->addItemModel(DEF_IN_GRP, DEF_IN_CLOUD, CT_Scene::staticGetType(), tr("Point Cloud"));
    res_model->addGroupModel("", DEF_IN_SCENE, CT_AbstractItemGroup::staticGetType(), tr("Input Scene Group"), "", CT_InAbstractGroupModel::CG_ChooseOneIfMultiple);
    res_model->addItemModel(DEF_IN_SCENE, DEF_IN_SCENE_CLOUD, CT_Scene::staticGetType(), tr("Input Scene"));
}

void SF_Euclidean_Clustering_Step::createPostConfigurationDialog() {
    CT_StepConfigurableDialog *config_dialog = newStandardPostConfigurationDialog();
    config_dialog->addDouble("The cloud is firstly downscaled with voxel size  ", " (m).", 0.01, 10, 4, _voxelSize);
    config_dialog->addDouble("Than an euclidean clustering routine is performed with threshold  ", " (m)." , 0.01, 10, 4, _euclideanDistance);
    config_dialog->addInt("A cluster has to contain at minimum ", " points.", 1, 9999, _minPts);
    createPostConfigurationDialogCitation(config_dialog);
}

void SF_Euclidean_Clustering_Step::createOutResultModelListProtected() {
    CT_OutResultModelGroupToCopyPossibilities *res_modelw = createNewOutResultModelToCopy(DEF_IN_RESULT);
    if(res_modelw != NULL) {
        res_modelw->addGroupModel(DEF_IN_SCENE, _out_grp_cluster, new CT_StandardItemGroup(), tr ("Euclidean Clustering") );
        res_modelw->addItemModel(_out_grp_cluster, _out_cloud_cluster, new CT_Scene(), tr("Tree seeds"));
    }
}

void SF_Euclidean_Clustering_Step::compute() {
    const QList<CT_ResultGroup*> &out_result_list = getOutResultList();
    CT_ResultGroup * out_result = out_result_list.at(0);
    identify_and_remove_corrupted_scenes(out_result);

    CT_ResultGroupIterator out_res_it(out_result, this, DEF_IN_GRP);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCL(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<size_t> indices;
    bool first = true;
    Eigen::Vector3d centerOfMass;

    while(!isStopped() && out_res_it.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) out_res_it.next();
        const CT_AbstractItemDrawableWithPointCloud* ct_cloud = (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD);
        if(first) {
            centerOfMass = ct_cloud->getCenterCoordinate();
            first = false;
        }
        CT_PointIterator iter(ct_cloud->getPointCloudIndex());
        while(iter.hasNext() && ! isStopped()) {
            iter.next();
            size_t index = iter.currentGlobalIndex();
            CT_Point ctp = iter.currentPoint();
            indices.push_back(index);
            pcl::PointXYZI p;
            p.x = ctp[0] - centerOfMass[0];
            p.y = ctp[1] - centerOfMass[1];
            p.z = ctp[2] - centerOfMass[2];
            std::cout << p.x << "; " << p.y << "; " << p.z << std::endl;
            cloudPCL->push_back(p);
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPCLDownscaled(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud (cloudPCL);
    sor.setLeafSize (_voxelSize, _voxelSize, _voxelSize);
    sor.filter (*cloudPCLDownscaled);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance (_euclideanDistance);
    ec.setMinClusterSize (1);
    ec.setMaxClusterSize (std::numeric_limits<int>::max());
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloudPCLDownscaled);
    ec.extract (cluster_indices);

    int index = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
          cloudPCLDownscaled->points[*pit].intensity = index;
      index++;
    }

    std::vector<CT_PointCloudIndexVector *> indexVec;
    for(size_t i = 0; i < cluster_indices.size(); i++) {
        CT_PointCloudIndexVector *mergedClouds = new CT_PointCloudIndexVector();
        indexVec.push_back(mergedClouds);
    }

    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZI>);
    kdtree->setInputCloud (cloudPCLDownscaled);
    for(size_t i = 0; i < cloudPCL->points.size(); i++) {
        pcl::PointXYZI point = cloudPCL->points[i];
        std::vector<int>   pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        if ( kdtree->nearestKSearch (point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
            int index = cloudPCLDownscaled->points[pointIdxNKNSearch[0] ].intensity;
            std::cout << index << std::endl;
            indexVec[index]->addIndex(indices[i]);
            std::cout << indices[i] << std::endl;
        }
    }

    CT_ResultGroupIterator out_res_it2(out_result, this, DEF_IN_SCENE);
    while(!isStopped() && out_res_it2.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) out_res_it2.next();
        for(size_t i = 0; i < indexVec.size(); i++) {
            if(indexVec[i]->size() > _minPts ) {
                CT_StandardItemGroup* cloud_grp = new CT_StandardItemGroup(_out_grp_cluster.completeName(), out_result );
                group->addGroup(cloud_grp);
                indexVec[i]->setSortType(CT_PointCloudIndexVector::SortedInAscendingOrder);
                CT_Scene* outScene = new CT_Scene(_out_cloud_cluster.completeName(), out_result, PS_REPOSITORY->registerPointCloudIndex(indexVec[i]) );
                outScene->updateBoundingBox();
                cloud_grp->addItemDrawable(outScene);
            }
        }
    }
}

void SF_Euclidean_Clustering_Step::createParamList(CT_ResultGroup * out_result) {
    adapt_parameters_to_expert_level();
    CT_ResultGroupIterator out_res_it(out_result, this, DEF_IN_GRP);
    while(!isStopped() && out_res_it.hasNext()) {
        CT_StandardItemGroup* group = (CT_StandardItemGroup*) out_res_it.next();
        const CT_AbstractItemDrawableWithPointCloud* ct_cloud = (const CT_AbstractItemDrawableWithPointCloud*) group->firstItemByINModelName(this, DEF_IN_CLOUD);
        SF_Param_Euclidean_Clustering<SF_Point_N> param;
        param._log = PS_LOG;
        param._cellSize = _voxelSize;
        param._euclideanDistance = _euclideanDistance;
        param._minSize = _minPts;
        param._itemCpy_cloud_in = ct_cloud;
        param._grpCpy_grp = group;
        _param_list.append(param);
    }
}
